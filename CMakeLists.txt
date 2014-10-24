cmake_minimum_required(VERSION 2.8.3)
project(vertigo_slam)

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake-modules/")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  visualization_msgs
  )

find_package(Eigen3 REQUIRED)
find_package(SuiteSparse REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)
find_package(G2O REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES vertigo_g2o ${CSPARSE_LIBARIES} ${EIGEN3_LIBRARIES} ${G2O_LIBRARIES}
  CATKIN_DEPENDS roscpp 
  DEPENDS Eigen3 SuiteSparse G2O 
  )

###########
## Build ##
###########

include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
  ${CSPARSE_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${BOOST_INCLUDE_DIRS}
  ${G2O_INCLUDE_DIR}
  )

add_library(vertigo_g2o
  src/g2o/types_g2o_robust.cpp
  src/g2o/edge_switchPrior.cpp
  src/g2o/edge_se2Switchable.cpp
  src/g2o/edge_se2MaxMixture.cpp
  src/g2o/edge_se3Switchable.cpp
  src/g2o/vertex_switchLinear.cpp
  )
target_link_libraries(vertigo_g2o ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES} ${G2O_LIBRARIES} )

add_executable(vertigo_load_dataset examples/robustG2O/load_dataset_test.cpp)
target_link_libraries(vertigo_load_dataset ${catkin_LIBRARIES} ${CSPARSE_LIBRARY} ${G2O_LIBRARIES} vertigo_g2o)

add_executable(vertigo_incremental_optimization examples/robustG2O/incremental_optimization.cpp)
target_link_libraries(vertigo_incremental_optimization ${catkin_LIBRARIES} ${CSPARSE_LIBRARY} ${G2O_LIBRARIES} vertigo_g2o)

#############
## Install ##
#############

install(DIRECTORY include/ DESTINATION include)

install(TARGETS vertigo_g2o vertigo_load_dataset vertigo_incremental_optimization
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

