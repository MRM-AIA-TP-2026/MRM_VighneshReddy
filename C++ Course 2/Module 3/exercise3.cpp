#include <iostream>
#include <string>

using namespace std;

int main(int argc, char** argv) {
  string original = (argv[1]);
  string modified;

  for (char c : original) {
    if (isupper(c)) {
      modified += 'u';
    } else if (islower(c)) {
      modified += 'l';
    } else {
      modified += '-';
    }
  }

  cout << original << endl;
  cout << modified << endl;

  return 0;
}