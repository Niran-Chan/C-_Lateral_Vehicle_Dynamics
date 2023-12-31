cmake_minimum_required(VERSION 3.27.1)

project(lateralDynamics)

# Set the C++ standard for the entire project

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(fmt 10.1)
find_package(Eigen3 3.4 REQUIRED NO_MODULE)
 
#find_package (Eigen3 3.4 REQUIRED NO_MODULE)
add_executable("${PROJECT_NAME}" main.cpp)
target_link_libraries(${PROJECT_NAME} fmt::fmt Eigen3::Eigen)


