#include <Eigen/Dense>
#include <chrono>
#include <iostream>
#include <memory>
#include <thread>

#include "robot_plan_runner/lcmt_test.hpp"

using namespace std;

void f0() {
  using namespace std::chrono_literals;
  this_thread::sleep_for(1us);
  //  throw runtime_error("f0 done");
}

using DoubleSeconds = std::chrono::duration<double, std::ratio<1, 1>>;

int main() {
  auto t_1 = std::chrono::high_resolution_clock::now();
  double t_1d =
      std::chrono::duration_cast<DoubleSeconds>(t_1.time_since_epoch()).count();
  f0();
  auto t_2 = std::chrono::high_resolution_clock::now();
  double t_2d =
      std::chrono::duration_cast<DoubleSeconds>(t_2.time_since_epoch()).count();

  cout << (t_2 - t_1).count() << endl;
  cout << t_2d - t_1d << endl;
  robot_plan_runner::lcmt_test test_msg;
  return 0;
}
