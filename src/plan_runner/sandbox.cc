#include <chrono>
#include <memory>
#include <Eigen/Dense>
#include <iostream>

using namespace std;

int main() {
  std::unique_ptr<Eigen::VectorXd> q(nullptr);
  cout << (q == nullptr) << endl;
  q = std::make_unique<Eigen::VectorXd>(3);
  cout << q->transpose() << endl;
  q.reset();
  cout << (q == nullptr) << endl;

  {
    Eigen::VectorXd p(3);
    p << 1, 2, 3;
    q = std::make_unique<Eigen::VectorXd>(p);
  }
  cout << *q << endl;
  return 0;
}
