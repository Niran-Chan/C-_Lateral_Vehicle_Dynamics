cmake_minimum_required(VERSION 3.27.1)

project(lateralDynamics)

# Set the C++ standard for the entire project

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable("${PROJECT_NAME}" main.cpp)


