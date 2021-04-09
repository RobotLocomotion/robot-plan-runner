#!/usr/bin/env bash

#git clone https://github.com/RobotLocomotion/robot-plan-runner.git
# build
mkdir /robot-plan-runner-build && cd /robot-plan-runner-build
cmake -DCMAKE_PREFIX_PATH=/opt/drake /robot-plan-runner
make -j

# run test
ctest
exit_status=$?
if [ ! $exit_status -eq 0 ]; then
  echo "Error code in running unit tests: " $exit_status
  exit $exit_status
fi
