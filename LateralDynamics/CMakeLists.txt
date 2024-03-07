cmake_minimum_required(VERSION 3.27.1)

project(lateralDynamics)

# Set the C++ standard for the entire project

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

#find_package(fmt 10.1)
#find_package(Eigen3 3.4 REQUIRED NO_MODULE)


find_package(Eigen3 REQUIRED)
#include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(external/eigen)

set(SOURCES HelperFunctions.cpp Models.cpp SimulateSystem.cpp Vehicle.cpp)
set(HEADERS HelperFunctions.hpp Models.hpp SimulateSystem.hpp Vehicle.hpp)

add_executable("${PROJECT_NAME}" main.cpp ${SOURCES} ${HEADERS})
#target_link_libraries(${PROJECT_NAME} fmt::fmt Eigen3::Eigen)


