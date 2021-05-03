#include <Eigen/Dense>
#include <chrono>
#include <iostream>
#include <memory>
#include <thread>
#include <vector>

#include "robot_plan_runner/lcmt_plan_status.hpp"
#include "robot_plan_runner/lcmt_plan_status_constants.hpp"

using namespace std;

struct A {
  A() {
    cout << "Constructor called" << flush << endl;
  }

  A(const A& a) {
    cout << "Copy constructor called" << flush << endl;
    n = a.n;
  }

  A(A&& a) noexcept {
    cout << "Move constructor called" << flush << endl;
    n = a.n;
  }

  A& operator=(const A& a) {
    cout << "Copy assignment called" << flush << endl;
    n = a.n;
    return *this;
  }

  A& operator=(A&& a) noexcept {
    cout << "Move assignment called" << flush << endl;
    n = a.n;
    return *this;
  }

  vector<int> n;
};


A f0() {
  cout << "f0 begin" << endl;
  A a;
  a.n = {10};
  cout << "f0 end" << endl;
  return std::move(a);
}


int main() {
  A a;
  a = f0();
  cout << a.n.size() << endl;


  robot_plan_runner::lcmt_plan_status plan_status_msg;
  robot_plan_runner::lcmt_plan_status_constants plan_status_constants;

  plan_status_msg.utime = 0;
  plan_status_msg.status = plan_status_constants.FINISHED;
  return 0;
}
