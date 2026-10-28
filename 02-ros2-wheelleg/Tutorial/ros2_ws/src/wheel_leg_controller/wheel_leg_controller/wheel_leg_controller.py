"""轮腿控制器（阶段 3）：VMC 五连杆 + 腿长 PID。

移植自 Wheel_Leg/LQR_VMC/：
  vmc_cal.py        五连杆正运动学 + VMC 力矩映射（原样复用）
  lqr_update.py     LQR 状态更新 + K 矩阵多项式（阶段 4 用）
  原版主循环 robot_control_main.py 只作参考，不整段搬运（作业 5.4 第 4 条）

本节点只做三件事：
  1. 订阅 /imu、/joint_states，解出姿态与关节量；
  2. 用五连杆正运动学算 L_0 / theta；
  3. 对 L_0 做 PID，经 VMC 映射成 4 个腿关节力矩，发 /actuator_cmd。

默认开启 LQR：阶段 4 是必做项，关闭可用 -p use_lqr:=false。
"""

import math

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rm_msgs.msg import WheelLegCmd
from sensor_msgs.msg import Imu, JointState

from .lqr_update import lqr_controller
from .vmc_cal import five_links_param

DT = 0.002
WHEEL_R = 0.06
TORQUE_LIMIT = 1.5          # 轮子力矩限幅（原版用的就是 1.5，比模型 ±2.0 更保守）
PUSH_LIMIT = 2.0            # 扰动脉冲限幅，取模型 ctrlrange 上限
MAX_I = 0.14                # 腿长 PID 输出上限 (N)

# joint_states 顺序：right1, right2, left1, left2, rightwheel, leftwheel
I_R1, I_R2, I_L1, I_L2, I_RW, I_LW = 0, 1, 2, 3, 4, 5


class PIDController:
    """原版 PIDController，逐行保持一致（integral 按步累加，不乘 dt）。"""

    def __init__(self, kp, ki, kd, max_out, max_i, feedforward=0.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_out = max_out
        self.max_i = max_i
        self.feedforward = feedforward
        self.integral = 0.0
        self.prev_error = 0.0

    def compute(self, current, target):
        error = target - current
        self.integral += error
        self.integral = np.clip(self.integral, -self.max_i, self.max_i)
        derivative = error - self.prev_error
        self.prev_error = error
        output = (self.kp * error + self.ki * self.integral
                  + self.kd * derivative + self.feedforward)
        return np.clip(output, -self.max_out, self.max_out)


class WheelLegController(Node):
    def __init__(self):
        super().__init__('wheel_leg_controller')

        self.declare_parameter('target_length', 0.10)   # 目标腿长 (m)
        self.declare_parameter('use_lqr', True)        # 默认开；-p use_lqr:=false 可退回阶段3
        self.target_length = float(self.get_parameter('target_length').value)
        self.use_lqr = bool(self.get_parameter('use_lqr').value)

        # ---- 抗扰测试（加分项）：push_t 秒后给两轮一个同向力矩脉冲 ----
        # 脉冲施加在轮子而非机身，物理上等价于瞬间横向冲量，恢复完全由 LQR 完成。
        self.declare_parameter('push_t', -1.0)         # 触发时刻 (s)，<0 = 不触发
        self.declare_parameter('push_torque', 1.0)     # 脉冲力矩 (N·m)
        self.declare_parameter('push_dur', 0.15)       # 脉冲时长 (s)
        self.push_t = float(self.get_parameter('push_t').value)
        self.push_torque = float(self.get_parameter('push_torque').value)
        self.push_dur = float(self.get_parameter('push_dur').value)
        self.get_logger().info(
            f'抗扰设置: push_t={self.push_t}, torque={self.push_torque} N·m, '
            f'dur={self.push_dur} s')

        # ---- 话题 ----
        self.cmd_pub = self.create_publisher(WheelLegCmd, 'actuator_cmd', 10)
        self.create_subscription(JointState, 'joint_states', self.on_joint, 10)
        self.create_subscription(Imu, 'imu', self.on_imu, 10)
        self.create_subscription(Twist, 'cmd_vel', self.on_cmd_vel, 10)

        # ---- 控制器实例：左右腿各一套，状态互不共享 ----
        self.fl_left = five_links_param(dt=DT)
        self.fl_right = five_links_param(dt=DT)
        self.lqr_left = lqr_controller(dt=DT)
        self.lqr_right = lqr_controller(dt=DT)
        self.l0_pid_left = PIDController(700.0, 0.0, 5.0, 150.0, 0.0)
        self.l0_pid_right = PIDController(700.0, 0.0, 5.0, 150.0, 0.0)
        self.yaw_pid_left = PIDController(0.5, 0.0, 1.0, 2.0, 0.0)
        self.yaw_pid_right = PIDController(0.5, 0.0, 1.0, 2.0, 0.0)

        # ---- 传感器缓存 ----
        self.ready = False
        self.pitch = 0.0
        self.gyro_pitch = 0.0
        self.gyro_yaw = 0.0
        self.pos = [0.0] * 6
        self.vel = [0.0] * 6

        # ---- 速度指令（阶段 2 的 /cmd_vel 继续沿用）----
        self.target_v = 0.0
        self.current_v = 0.0
        self.target_w = 0.0

        self.t = 0.0
        self.steps = 0
        self.create_timer(DT, self.tick)
        self.get_logger().info(
            f'阶段3 已启动：VMC，target_length={self.target_length} m，'
            f'use_lqr={self.use_lqr}')

    # ================= 订阅回调 =================
    def on_joint(self, msg):
        if len(msg.position) < 6 or len(msg.velocity) < 6:
            return
        self.pos = list(msg.position[:6])
        self.vel = list(msg.velocity[:6])
        self.ready = True

    def on_imu(self, msg):
        """四元数 → 欧拉角，与 robot_info_update.py 完全同一套公式。"""
        q = msg.orientation
        w, x, y, z = q.w, q.x, q.y, q.z

        t0 = 2.0 * (w * x + y * z)
        t1 = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)

        t2 = 2.0 * (w * y - z * x)
        t2 = max(-1.0, min(1.0, t2))
        pitch = math.asin(t2)

        # 原版对 pitch / gyro_pitch 都取负，这是传感器坐标系约定的适配，
        # 不是"修正 bug"——改了必然发散
        self.pitch = -pitch
        self.gyro_pitch = -float(msg.angular_velocity.y)
        self.gyro_yaw = float(msg.angular_velocity.z)

    def on_cmd_vel(self, msg):
        self.target_v = float(msg.linear.x)
        self.target_w = float(msg.angular.z)

    # ================= 主循环 =================
    def tick(self):
        self.t += DT
        self.steps += 1

        cmd = WheelLegCmd()
        if not self.ready:
            # 还没收到状态就发零力矩：仿真节点是"指令驱动"的，不发它就不动
            self.cmd_pub.publish(cmd)
            return

        # ---- 共模量：两轮的"平均前进"与"平均转动" ----
        common_speed = (self.vel[I_LW] - self.vel[I_RW]) / 2.0
        common_pos = (self.pos[I_LW] - self.pos[I_RW]) / 2.0

        # ---- 速度指令爬坡（原版 VelocityRampFilter：加速 0.8、减速 1.4 m/s²）----
        rate = 0.8 if abs(self.target_v) > abs(self.current_v) else 1.4
        dv = self.target_v - self.current_v
        step = rate * DT
        self.current_v += max(-step, min(step, dv))

        # ---- x_b_set：只在运动时跟随，站桩时冻结（第 9 章 5.9 的坑）----
        if not (self.current_v == 0.0 and self.target_v == 0.0):
            self.lqr_left.x_b_set = common_pos * WHEEL_R
            self.lqr_right.x_b_set = common_pos * WHEEL_R

        self.lqr_left.v_set = self.current_v
        self.lqr_right.v_set = self.current_v

        # ---- 左腿 ----
        ctrl_l0, ctrl_l1, left_wheel_T, l0_l = self.control_leg(
            leg_flag=1, fl=self.fl_left, lqr=self.lqr_left,
            l0_pid=self.l0_pid_left, yaw_pid=self.yaw_pid_left,
            m1=self.pos[I_L1], m2=self.pos[I_L2],
            common_pos=common_pos, common_speed=common_speed, sign=+1)

        # ---- 右腿 ----
        ctrl_r0, ctrl_r1, right_wheel_T, l0_r = self.control_leg(
            leg_flag=0, fl=self.fl_right, lqr=self.lqr_right,
            l0_pid=self.l0_pid_right, yaw_pid=self.yaw_pid_right,
            m1=self.pos[I_R1], m2=self.pos[I_R2],
            common_pos=common_pos, common_speed=common_speed, sign=-1)

        # ---- 落地到执行器顺序（robot.xml 的 ctrl 顺序）----
        cmd.right_phi1 = ctrl_r0
        cmd.right_phi4 = ctrl_r1
        cmd.left_phi1 = ctrl_l0
        cmd.left_phi4 = ctrl_l1
        cmd.right_wheel = right_wheel_T
        cmd.left_wheel = left_wheel_T

        # ---- 抗扰测试：脉冲期间两轮同向加力矩（前向冲量）----
        if 0.0 <= self.push_t <= self.t < self.push_t + self.push_dur:
            cmd.left_wheel = float(np.clip(cmd.left_wheel + self.push_torque, -PUSH_LIMIT, PUSH_LIMIT))
            cmd.right_wheel = float(np.clip(cmd.right_wheel - self.push_torque, -PUSH_LIMIT, PUSH_LIMIT))

        self.cmd_pub.publish(cmd)

        if self.steps % 500 == 0:
            self.get_logger().info(
                f'sim_t={self.t:6.2f}s  L0_l={l0_l:.4f} L0_r={l0_r:.4f} m  '
                f'pitch={math.degrees(self.pitch):+6.2f}°  '
                f'th_l={math.degrees(self.fl_left.theta):+6.2f}°  '
                f'v_meas={self.current_v:+.2f}/{common_speed * WHEEL_R:+.2f}')

    # ================= 单腿控制 =================
    def control_leg(self, leg_flag, fl, lqr, l0_pid, yaw_pid, m1, m2,
                    common_pos, common_speed, sign):
        # 1. 正运动学：从两个髋关节角解出 L_0 / phi_0 / theta
        fl.forward_kinematics_cal(leg_flag, m1, m2, self.pitch, self.gyro_pitch)
        lqr.states_update(leg_flag, fl, self.pitch, self.gyro_pitch,
                          sign * common_pos, sign * common_speed, WHEEL_R)
        lqr.calc_k_matrix_from_poly(fl.L_0)

        # 2. 腿长 PID → 轴向虚拟力 F_0
        f0_ctrl = l0_pid.compute(fl.L_0, self.target_length)
        cos_theta = math.cos(fl.theta)
        # 4/cos(theta) 是重力前馈（两条腿各分担一点），不是随便写的常数
        f0 = (4.0 / cos_theta if cos_theta != 0.0 else 0.0) + f0_ctrl
        f0 = float(np.clip(f0, -100.0, 100.0))

        # 3. 髋关节虚拟力矩 T_p：阶段 3 为 0，阶段 4 由 LQR 给出
        if self.use_lqr:
            t_p = (lqr.K[1] * lqr.theta
                   + lqr.K[3] * lqr.d_theta
                   + lqr.K[5] * (lqr.x_b - lqr.x_b_set)
                   + lqr.K[7] * (lqr.v_b_whole - lqr.v_set)
                   + lqr.K[9] * lqr.phi
                   + lqr.K[11] * lqr.d_phi)

            wheel_T = (lqr.K[0] * lqr.theta
                       + lqr.K[2] * lqr.d_theta
                       + lqr.K[4] * (lqr.x_b - lqr.x_b_set)
                       + lqr.K[6] * (lqr.v_b_whole - lqr.v_set)
                       + lqr.K[8] * lqr.phi
                       + lqr.K[10] * lqr.d_phi)
            yaw_ctrl = yaw_pid.compute(self.gyro_yaw, self.target_w)
            # 左右轮的符号约定与原版一致：左 -，右 +
            wheel = wheel_T if leg_flag == 0 else -wheel_T
            wheel = float(np.clip(wheel - yaw_ctrl, -TORQUE_LIMIT, TORQUE_LIMIT))
        else:
            t_p = 0.0
            wheel = 0.0

        # 4. VMC 力矩映射 → 两个髋关节力矩
        t0, t1 = fl.VMC_torque_cal(f0, t_p)
        if leg_flag == 1:            # 左腿取负，原版的符号适配
            t0, t1 = -t0, -t1

        return float(t0), float(t1), wheel, fl.L_0


def main(args=None):
    rclpy.init(args=args)
    node = WheelLegController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
