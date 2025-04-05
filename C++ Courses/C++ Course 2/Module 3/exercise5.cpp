#include <iostream>
using namespace std;

int main(int argc, char** argv) {
  
  string my_string = (argv[1]);
  
  //add code below this line

cout << "Enter a string with an even number of characters: ";
    cin >> my_string;

    
    if (my_string.length() % 2 != 0) {
        cout << "Error: String must have an even number of characters." << endl;
        return 1;
    }

   
    for (int i = 0; i < my_string.length(); i += 2) {
        swap(my_string[i], my_string[i + 1]);
    }

    cout << "Swapped string: " << my_string << endl;


  //add code above this line
  
  return 0;
  
}