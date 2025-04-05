#include <iostream>
using namespace std;

//add function definitions below this line
int RecursiveSum(int x){
  if (x==0){
    return x;
  }
  return x + RecursiveSum(x-1);
}
//add function definitions above this line

int main(int argc, char** argv) {
  cout << RecursiveSum(stoi(argv[1])) << endl;
  return 0;
}
