#!/usr/bin/env bash
set -euxo pipefail

# build plan runner
mkdir /robot-plan-runner-build && cd /robot-plan-runner-build
cmake -DCMAKE_PREFIX_PATH=/opt/drake  /github/workspace
make -j
export PYTHON_PATH=/robot-plan-runner-build/robot-plan-runner-lcmtypes

# run tests
ctest -V .
