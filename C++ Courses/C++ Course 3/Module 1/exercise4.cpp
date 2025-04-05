#include <iostream>
#include <vector>
using namespace std;

//add code below this line

bool IsPalindrome(string s){
  for (int i =0; i<s.length()/2; i++){
    if (s[i]!=s[s.length()-i-1]){
      return false;
    }
  }
  return true;
}

//add code above this line

int main(int argc, char** argv) {
  string x = argv[1];
  cout << boolalpha << IsPalindrome(x) << endl;
}