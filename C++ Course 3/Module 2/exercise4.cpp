#include <iostream>
using namespace std;

//add function definitions below this line
string ReverseString(string str) {
    if (str.empty()) {
        return ""; 
    } else {
        return ReverseString(str.substr(1)) + str[0];
    }
}
//add function definitions above this line

int main(int argc, char** argv) {
  cout << ReverseString(argv[1]) << endl;
  return 0;
}
