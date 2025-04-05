#include <iostream>
#include <fstream>
#include <vector>
#include <sstream>
#include <iomanip>
using namespace std;

int main(int argc, char** argv) {

////////// DO NOT EDIT! //////////
  string path = argv[1];        //
//////////////////////////////////  
  
  //add code below this line
    struct Person {
    string name;
    int age;
    string occupation;


    ifstream file(path);

    if (!file.is_open()) {
        cerr << "Error: Could not open file '" << path << "'" << endl;
        return 1;
    }

    vector<Person> people;
    string line;
    while (getline(file, line)) {
        stringstream ss(line);
        Person person;
        getline(ss, person.name, '\t');
        ss >> person.age;
        getline(ss, person.occupation, '\t');
        people.push_back(person);
    }

    file.close();

    Person oldestPerson = people[0];
    for (const Person& person : people) {
        if (person.age > oldestPerson.age) {
            oldestPerson = person;
        }
    }

    cout << "The oldest person is: " << oldestPerson.name << endl;

    return 0;
}

  //add code above this line
  
  return 0;
  
}
