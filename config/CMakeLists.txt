# Copy the default yaml file into the build directory so that the user doesn't
# always have to provide config files through argv.
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/default.yaml
    DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
