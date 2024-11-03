#include <iostream>
using namespace std;

int main(int argc, char** argv) {
  
  string my_string = (argv[1]);
  
  //add code below this line

int length = my_string.length();
for (int i = 0; i < length; i++) {
    for (int j = 0; j < length; j++) {
      cout << my_string << " ";
    }
    cout << endl;
  }

  //add code above this line
  
  return 0;
  
}