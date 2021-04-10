#! /bin/bash
set -euxo pipefail
export APT_KEY_DONT_WARN_ON_DANGEROUS_USAGE=1 DEBIAN_FRONTEND=noninteractive

apt-get install -o APT::Acquire::Retries=4 -o Dpkg::Use-Pty=0 -qy --no-install-recommends xvfb ca-certificates libzmq3-dev libgtest-dev libgflags-dev libpgm-dev
trap 'rm -rf /var/cache/apt/archives/*.deb /var/cache/apt/archives/partial/*.deb /var/cache/apt/*.bin /var/lib/apt/lists/* /var/log/apt/*' EXIT

pushd /opt
wget -nv https://drake-packages.csail.mit.edu/drake/nightly/drake-latest-bionic.tar.gz
trap 'rm -rf /opt/drake-latest-bionic.tar.gz /var/cache/apt/archives/*.deb /var/cache/apt/archives/partial/*.deb /var/cache/apt/*.bin /var/lib/apt/lists/* /var/log/apt/*' EXIT
tar -xf drake-latest-bionic.tar.gz
popd

#apt-get install -o APT::Acquire::Retries=4 -o Dpkg::Use-Pty=0 -qy --no-install-recommends $(cat /opt/drake/share/drake/setup/packages-bionic.txt)

# install cppzmq
curl -o cppzmq.tar.gz -L https://github.com/zeromq/cppzmq/archive/refs/tags/v4.7.1.tar.gz
tar -xzf cppzmq.tar.gz
pushd ./cppzmq-4.7.1

mkdir build
pushd build
cmake -DCPPZMQ_BUILD_TESTS=OFF ..
make install

popd
popd
