// Venom Algorithm Group Assignment 1: fix password_checker.cpp
//
// Analysis of the three checks:
// 1) Digit check: contains_between(password, '0', '9') uses a closed interval
//    [lower, upper], so '0'..'9' is judged correctly; no change needed.
// 2) Length check: the original code password.size() <= 10 contradicted the
//    rule "at least 10 characters": short passwords passed and long ones failed.
//    Fixed to password.size() >= 10.
// 3) Range boundaries: contains_between uses lower <= ch && ch <= upper
//    (inclusive), covering endpoints correctly; no change needed.
//
// Custom tests (expected output):
//   Li Wang Aa12345678 -> VALID   (10 chars, upper+lower+digit, no name)
//   Li Wang aaaaaaaa1  -> INVALID (no uppercase)
//   Li Wang AAAAAAAA1  -> INVALID (no lowercase)
//   Li Wang Aa1234567  -> INVALID (length 9, shorter than 10)
//   Li Wang AaWang123  -> INVALID (contains last name Wang)
#include <iostream>
#include <string>

bool contains_between(const std::string& text, char lower, char upper) {
    for (char ch : text) {
        if (lower <= ch && ch <= upper) {
            return true;
        }
    }
    return false;
}

bool check_password(const std::string& first_name,
                    const std::string& last_name,
                    const std::string& password) {
    bool length_ok = password.size() >= 10;
    bool upper_ok = contains_between(password, 'A', 'Z');
    bool lower_ok = contains_between(password, 'a', 'z');
    bool digit_ok = contains_between(password, '0', '9');
    bool name_ok = (first_name.empty() || password.find(first_name) == std::string::npos) &&
                   (last_name.empty() || password.find(last_name) == std::string::npos);
    return length_ok && upper_ok && lower_ok && digit_ok && name_ok;
}

int main() {
    std::string first_name, last_name, password;
    if (!(std::cin >> first_name >> last_name >> password)) {
        std::cerr << "请输入：名 姓 密码\n";
        return 1;
    }
    std::cout << (check_password(first_name, last_name, password)
                      ? "VALID" : "INVALID") << '\n';
    return 0;
}
