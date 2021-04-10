#!/usr/bin/env bash
set -euxo pipefail

# install cppzmq
curl -o cppzmq.tar.gz -L https://github.com/zeromq/cppzmq/archive/refs/tags/v4.7.1.tar.gz
sudo tar -xzf cppzmq.tar.gz -C / && rm cppzmq.tar.gz

pushd /cppzmq-4.7.1
mkdir build
pushd build
cmake -DCPPZMQ_BUILD_TESTS=OFF ..
make install

popd
popd
