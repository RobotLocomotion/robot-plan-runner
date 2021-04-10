#! /bin/bash
export PYTHONPATH=/opt/drake/lib/python3.6/site-packages:$PYTHONPATH
set -euxo pipefail

mkdir ./build
pushd build

cmake -DCMAKE_PREFIX_PATH=/opt/drake ..
make -j
ctest -V .

popd
rm -rf build
