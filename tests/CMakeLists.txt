cmake_minimum_required(VERSION 3.14)

project(MyProject C CXX)

set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 23)

enable_testing()

# Add gtest
add_subdirectory(googletest)

# Create test target
add_executable(my_tests
    ./main.cpp
)

target_link_libraries(my_tests
    gtest
    gtest_main
    pthread  # Needed on Linux
)

include(GoogleTest)
gtest_discover_tests(my_tests)
