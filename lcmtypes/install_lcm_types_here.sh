#!/usr/bin/env bash
set -euxo pipefail

pushd /github/workspace/lcmtypes
mkdir build
pushd build

cmake -DCMAKE_PREFIX_PATH=/opt/drake ..
make robot-plan-runner-lcm-types.sources
make install

popd
popd
