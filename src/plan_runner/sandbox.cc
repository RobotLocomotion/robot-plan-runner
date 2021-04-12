#include <Eigen/Dense>
#include <iostream>

using namespace std;

int main() {
  Eigen::VectorXd v;
  v.resize(3);
  v << 0, 1, 2;
  cout << v.array().isNaN().sum() << endl;
  return 0;
}
