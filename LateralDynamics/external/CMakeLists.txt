#Cmake Package Manager for Dependencies

include(../cmake/CPM.cmake)
CPMAddPackage("gl:libeigen/eigen#3.4.0")

# link dependencies
#include_directories(${Eigen_INCLUDE_DIRS})
link_directories(${Eigen_INCLUDE_DIRS})