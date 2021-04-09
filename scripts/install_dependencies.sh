#!/usr/bin/env bash

# install cppzmq
curl -o cppzmq.tar.gz -L https://github.com/zeromq/cppzmq/archive/refs/tags/v4.7.1.tar.gz
sudo tar -xzf cppzmq.tar.gz -C /
cd /cppzmq-4.7.1 && mkdir build && cd build
cmake -DCPPZMQ_BUILD_TESTS=OFF ..
make install


