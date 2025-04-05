#include <iostream>
#include <vector>
using namespace std;

int main(int argc, char** argv) {
  
  vector<string> reverse(0);
  reverse.push_back(argv[1]);
  reverse.push_back(argv[2]);
  reverse.push_back(argv[3]);
  
  //add code below this line

reverse.push_back(argv[3]);
reverse.push_back(argv[2]);
reverse.push_back(argv[1]);
reverse.erase(reverse.begin());
reverse.erase(reverse.begin());
reverse.erase(reverse.begin());

  //add code above this line
  
  for (auto a : reverse) {
    cout << a << endl;
  }
  
  return 0;
  
}