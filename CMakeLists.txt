cmake_minimum_required(VERSION 2.8.3)
project(vertigo_slam)

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake-modules/")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED COMPONENTS
  roscpp
  visualization_msgs
)

## System dependencies are found with CMake's conventions
find_package(Eigen3 REQUIRED)
find_package(SuiteSparse REQUIRED)
find_package(G2O REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)
message("G2O Libraries: " ${G2O_LIBRARIES})
###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES vertigo_g2o ${G2O_LIBRARIES} ${CSPARSE_LIBARIES} ${EIGEN3_LIBRARIES} 
  CATKIN_DEPENDS roscpp 
  DEPENDS G2O Eigen3 SuiteSparse 
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)

include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
  ${CSPARSE_INCLUDE_DIR}
  ${G2O_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${BOOST_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(vertigo_g2o 
  src/g2o/types_g2o_robust.cpp
  src/g2o/edge_switchPrior.cpp
  src/g2o/edge_se2Switchable.cpp
  src/g2o/edge_se2MaxMixture.cpp
  src/g2o/edge_se3Switchable.cpp
  src/g2o/vertex_switchLinear.cpp
)

## Declare a cpp executable
add_executable(vertigo_load_dataset examples/robustG2O/load_dataset_test.cpp)
add_executable(vertigo_incremental_optimization examples/robustG2O/incremental_optimization.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(vertigo_node vertigo_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(vertigo_g2o ${catkin_LIBRARIES} ${G2O_LIBRARIES} ${EIGEN3_LIBRARIES})
target_link_libraries(vertigo_load_dataset ${catkin_LIBRARIES} ${G2O_LIBRARIES} ${CSPARSE_LIBRARY} vertigo_g2o g2o_types_slam2d)
target_link_libraries(vertigo_incremental_optimization ${catkin_LIBRARIES} ${G2O_LIBRARIES} ${CSPARSE_LIBRARY} vertigo_g2o g2o_types_slam2d)

#############
## Install ##
#############

install(DIRECTORY include/ DESTINATION include)
install(TARGETS vertigo_g2o 
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    )



# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
#install(TARGETS vertigo vertigo_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_vertigo.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
