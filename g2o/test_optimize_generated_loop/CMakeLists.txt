cmake_minimum_required(VERSION 2.8)

project(EssentialTest)

set( CMAKE_CXX_COMPILER "g++" )
set( CMAKE_CXX_FLAGS "-std=c++11 -O3 " )

find_package( G2O REQUIRED )
find_package( Cholmod REQUIRED )
include_directories( ${G2O_INCLUDE_DIRS} )
find_package( CSparse REQUIRED )

#add_library(hello_shared SHARED libHelloSLAM.cpp)
INCLUDE_DIRECTORIES(
"/usr/include/eigen3"
    ${G2O_INCLUDE_DIRS}
    ${CSPARSE_INCLUDE_DIR}
    ${Cholmod_INCLUDE_DIR}
)

link_directories(
"/usr/local/lib"
)

link_libraries(
${Cholmod_LIBRARIES}
cholmod
g2o_types_slam3d
g2o_types_sba
g2o_core
g2o_stuff
g2o_solver_dense.so
g2o_solver_cholmod.so
)

add_executable(test test.cpp)
