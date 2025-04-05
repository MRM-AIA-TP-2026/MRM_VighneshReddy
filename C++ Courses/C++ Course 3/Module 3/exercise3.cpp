#include <iostream>
#include <vector>
using namespace std;

//add class definitions below this line
    class Superhero{
  public:
    string name;
    string identity;
    vector<string> powers;
  
  Superhero(string n, string i, vector<string> p){
    name = n;
    identity = i;
    powers = p;
  }
};
//add class definitions above this line

int main() {

  Superhero iron_man("Iron Man", "Tony Stark", {"armored suit", "laser beams", "flight"});
  cout << iron_man.name << endl;
  cout << iron_man.identity << endl;
  for (auto a : iron_man.powers) {
    cout << a << endl;
  }

  return 0;
  
}
