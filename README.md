# Robot Plan Runner
![ci_badge](https://github.com/robotlocomotion/robot-plan-runner/actions/workflows/ci.yml/badge.svg)

---
This repo requires cppzmq which can be installed by running
```
cd robot-plan-runner
sudo ./scripts/install_cppzmq.sh
```

This repo uses the [CMake `find_package(drake)` mechanism](https://github.com/RobotLocomotion/drake-external-examples/tree/master/drake_cmake_installed) to find an 
installed instance of Drake. 

To include Drake path as a flag in CMake use `-DCMAKE_PREFIX_PATH`. For example,

```
cd robot-plan-runner
mkdir build && cd build 
cmake -DCMAKE_PREFIX_PATH=/opt/drake ..
make -j4
```

To use lcm types defined within plan runner, add the python path:
`export PYTHONPATH={ROBOT_PLAN_RUNNER_DIR}/build/robot-plan-runner-lcmtypes:${PYTHONPATH}`

To use the client as an external module, add the python path:
`export PYTHONPATH={ROBOT_PLAN_RUNNER_DIR}/src:${PYTHONPATH}`


An incomplete design documentation can be found [here](https://slides.com/pang/deck-36762e).


## Continuous Integration
- CI is implemented by a custom Github Action that builds a docker image 
  using `./bionic.dockerfile` and runs the entrypoint script `.
  /sciprts/buld_test.sh`.
  
