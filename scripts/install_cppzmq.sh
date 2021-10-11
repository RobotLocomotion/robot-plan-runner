#!/usr/bin/env bash
set -euxo pipefail

if [[ "${EUID}" -ne 0 ]]; then
  echo 'ERROR: This script must be run as root' >&2
  exit 1
fi

# install libzmq
apt-get update
apt-get install libzmq3-dev

# install cppzmq
curl -o cppzmq.tar.gz -L https://github.com/zeromq/cppzmq/archive/refs/tags/v4.7.1.tar.gz
tar -xzf cppzmq.tar.gz -C /opt && rm cppzmq.tar.gz

pushd /opt/cppzmq-4.7.1
mkdir build
pushd build
cmake -DCPPZMQ_BUILD_TESTS=OFF ..
make install

popd
popd
