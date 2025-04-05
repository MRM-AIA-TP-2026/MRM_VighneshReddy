#include <iostream>
using namespace std;

int main(int argc, char** argv) {
  
  bool test = atoi((argv[1]));
  
  //add code below this line

bool my_bool = true;

cout << boolalpha << my_bool << endl;
my_bool = test;
cout << boolalpha << my_bool << endl;

  //add code above this line
  
  return 0;
  
}
