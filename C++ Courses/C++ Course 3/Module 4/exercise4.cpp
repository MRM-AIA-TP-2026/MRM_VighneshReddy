#include <iostream>
using namespace std;

//add class definitions below this line

class MarbleBag {
  public:
    MarbleBag(int r, int b, int y) {
      red = r;
      blue = b;
      yellow = y;
    }

  private:
    int red;
    int blue;
    int yellow;

  public:

    int AddRed(int n){
      red+=n;
      return red;
    }

    int AddBlue(int n){
      blue+=n;
      return blue;
    }

    int AddYellow(int n){
      yellow+=n;
      return yellow;
    }

    int RedTotal(){
      return red;
    }

    int YellowTotal(){
      return yellow;
    }

    int BlueTotal(){
      return blue;
    }

    int BagTotal(){
      return red+blue+yellow;
    }


};

//add class definitions above this line   

int main() {
  
  //DO NOT EDIT code below this line
  
  MarbleBag bag(12, 8, 19);
  bag.AddRed(4);
  bag.AddBlue(12);
  bag.AddYellow(-1);
  bag.AddBlue(-3);
  cout << "There are " << bag.RedTotal() << " red marbles." << endl;
  cout << "There are " << bag.BlueTotal() << " blue marbles." << endl;
  cout << "There are " << bag.YellowTotal() << " yellow marbles." << endl;
  cout << "There are " << bag.BagTotal() << " total marbles." << endl;

  //DO NOT EDIT code above this line
  
  return 0;
  
}