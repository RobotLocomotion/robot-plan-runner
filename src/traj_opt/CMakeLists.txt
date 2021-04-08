add_library(plan_base plan_base.h)
target_link_libraries(plan_base drake::drake)

add_executable(run_zmq_example run_zmq_example.cc)
target_link_libraries(run_zmq_example cppzmq)