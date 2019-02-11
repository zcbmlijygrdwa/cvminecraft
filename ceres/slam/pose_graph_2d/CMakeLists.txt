# CMake 工程，读者应该熟悉了，我就不一一注释了
cmake_minimum_required( VERSION 2.8 )
project( test_project )

set( CMAKE_BUILD_TYPE Release )
set( CMAKE_CXX_FLAGS "-std=c++11 -Wall -O2 -march=native" )

list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )

find_package( OpenCV REQUIRED )
find_package( Ceres REQUIRED )
find_package( Eigen3 REQUIRED )
find_package( Cholmod )
find_package( G2O REQUIRED )

include_directories( 
${EIGEN3_INCLUDE_DIR} 
${CHOLMOD_INCLUDE_DIR}
${CERES_INCLUDE_DIRS} 
)



add_executable(pose_graph_2d
  angle_local_parameterization.h
  normalize_angle.h
  pose_graph_2d.cc
  pose_graph_2d_error_term.h
  types.h)
target_link_libraries(pose_graph_2d ceres ${GFLAGS_LIBRARIES})

