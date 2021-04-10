# Robot Plan Runner
![ci_badge](https://github.com/robotlocomotion/robot-plan-runner/actions/workflows/ci.yml/badge.svg)

---
This repo uses the [CMake `find_package(drake)` mechanism](https://github.com/RobotLocomotion/drake-external-examples/tree/master/drake_cmake_installed) to find an 
installed instance of Drake.

## Continuous Integration
- CI is implemented by a custom Github Action that builds a docker image 
  using `./bionic.dockerfile` and runs the entrypoint script `.
  /sciprts/buld_test.sh`.
  