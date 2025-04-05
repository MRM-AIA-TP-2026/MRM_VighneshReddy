#include <iostream>
#include <vector>
#include<algorithm>
using namespace std;

//add function definitions below this line
int GetMax(const vector<int> nums, int index = 0) {
    if (index == nums.size() - 1) {
        return nums[index];
    }
    return max(nums[index], GetMax(nums, index + 1));
}
//add function definitions above this line

int main(int argc, char** argv) {
  vector<int> nums;
  for (int i = 1; i < argc; i++) {
    nums.push_back(stoi(argv[i]));
  }
  cout << GetMax(nums) << endl;
  return 0;
}
