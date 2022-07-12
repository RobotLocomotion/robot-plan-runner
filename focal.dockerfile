FROM ubuntu:20.04

# Install curl and useful transport
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && yes "Y" \
      | apt-get install --no-install-recommends curl apt-transport-https sudo \
      ca-certificates libgtest-dev libgflags-dev \
      && rm -rf /var/lib/apt/lists/* \
      && apt-get clean all

# install drake.
ENV DRAKE_URL=https://drake-packages.csail.mit.edu/drake/nightly/drake-latest-focal.tar.gz
RUN curl -o drake.tar.gz $DRAKE_URL
RUN tar -xzf drake.tar.gz -C /opt && rm drake.tar.gz
RUN apt-get update \
  && yes "Y" | bash /opt/drake/share/drake/setup/install_prereqs \
  && rm -rf /var/lib/apt/lists/* \
  && apt-get clean all

## install cppzmq
COPY scripts/install_cppzmq.sh /install_dependencies.sh
RUN /bin/bash /install_dependencies.sh

# put drake on the python path.
ENV PYTHONPATH /opt/drake/lib/python3.8/site-packages:$PYTHONPATH
