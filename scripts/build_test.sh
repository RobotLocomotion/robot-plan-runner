#!/usr/bin/env bash
set -euxo pipefail

# build lcm types
bash /github/workspace/lcmtypes/install_lcm_types_here.sh

# build plan runner
mkdir /robot-plan-runner-build && cd /robot-plan-runner-build
cmake -DCMAKE_PREFIX_PATH=/opt/drake  /github/workspace
make -j

# run tests
ctest -V .
