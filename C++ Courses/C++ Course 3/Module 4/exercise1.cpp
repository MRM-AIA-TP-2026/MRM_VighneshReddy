#include <iostream>
using namespace std;

//add class definitions below this line

class CelestialBody {
  public:
    CelestialBody(string n, double diam, double dist, int m) {
      name = n;
      diameter = diam;
      distance = dist;
      moons = m;
    }
    
    string name;
    double diameter;
    double distance;
    int moons;
};

//add class definitions above this line   

double ComparedToEarth(CelestialBody planet) {
  double earth = 12756.3;
  double relative_size = planet.diameter / earth;
  return relative_size;
}

int main() {
  
  CelestialBody planet("Jupiter", 142984, 778360000, 79);
  
  //Modify the code below
  
  cout << ComparedToEarth(planet) << endl;

  //Modify the code above

  return 0;
  
}