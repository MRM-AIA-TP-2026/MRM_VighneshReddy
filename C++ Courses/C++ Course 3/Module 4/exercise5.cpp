#include <iostream>
using namespace std;

//add class definitions below this line

class Subway {
  public:
    Subway() {
      passengers = 0;
      total_fares = 0;
    }
    
  private:
    const double fare = 2.40; //variable cannot be reassigned
    int passengers;
    double total_fares;

  public:

    int GetPassengers(){
      return passengers;
    }

    double GetFares(){
      return total_fares;
    }

    void Board(int n){
      if(n>=0){
        passengers+=n;
        total_fares+=CalculateFares(n);
      }
    }

    double CalculateFares(int n){
      return n*fare;
    }

    void Disembark(int n){
      if (n>0){
        passengers-=n;
      }
    }
};

//add class definitions above this line   

int main() {
  
  //DO NOT EDIT code below this line

  Subway s;
  cout << s.GetPassengers() << endl;
  s.Board(23);
  s.Disembark(12);
  cout << s.GetPassengers() << endl;
  cout << s.GetFares() << endl;

  //DO NOT EDIT code above this line
  
  return 0;
  
}