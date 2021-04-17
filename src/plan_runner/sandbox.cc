#include <chrono>
#include <memory>
#include <Eigen/Dense>
#include <iostream>
#include <thread>

using namespace std;

void f0() {
  using namespace std::chrono_literals;
  this_thread::sleep_for(1000ms);
//  throw runtime_error("f0 done");
}

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

  auto t = std::thread(f0);

  if (t.joinable()) {
    t.join();
  }
  cout << "main done." << endl;
  return 0;
}
