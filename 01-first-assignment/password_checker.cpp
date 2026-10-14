//if (lower < ch && ch < upper) { 原代码不包含边界导致错误
//bool digit_ok = contains_between(password, 0, 9);   原代码不是字符
//bool length_ok = password.size() <=10; 原代码长度判断太短且没有最低长度限制 
//int checked = 0; 未使用的变量
//主函数顺序错误，导致提示未输出，已改为输入错误时提示并且在初始时提示
//输入Li Wang 1234567890 输出INVALID
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
    bool length_ok = (password.size() >=10) && (password.size()<= 20);        //原代码长度判断太短且没有最低长度限制 
    bool upper_ok = contains_between(password, 'A', 'Z');
    bool lower_ok = contains_between(password, 'a', 'z');
    bool digit_ok = contains_between(password, '0', '9');
    bool name_ok = password.find(first_name) == std::string::npos &&
                   password.find(last_name) == std::string::npos;
  
    return length_ok && upper_ok && lower_ok && digit_ok && name_ok;
}

int main() {
    std::string first_name, last_name, password;
    std::cout << "请输入：名 姓 密码\n";
    if (!(std::cin >> first_name >> last_name >> password)) {
        std::cerr << "\n请输入：名 姓 密码\n";
        return 1;
    }
    std::cout << (check_password(first_name, last_name, password)
                      ? "VALID" : "INVALID") << '\n';
    return 0;
}
