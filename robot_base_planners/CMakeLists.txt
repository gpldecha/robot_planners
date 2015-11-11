cmake_minimum_required(VERSION 2.8.3)
project(robot_base_planners)

find_package(catkin REQUIRED roscpp statistics_ml kuka_action_server)

find_package(Armadillo REQUIRED)

set(CMAKE_CXX_FLAGS "-O2 -O3 -std=c++11 -mtune=native -fPIC -Wall")

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME} mlpack
  CATKIN_DEPENDS roscpp statistics_ml kuka_action_server
)


set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
set(INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)
set(SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)

include_directories(include ${catkin_INCLUDE_DIRS} ${INCLUDE_DIR})

set(H_FILES ${INCLUDE_DIR}/robot_planners/basePlanner.h
	    ${INCLUDE_DIR}/robot_planners/discretePlanner.h
	    ${INCLUDE_DIR}/robot_planners/gmmPlanner.h
)

set(S_FILES ${SRC_DIR}/basePlanner.cpp
            ${SRC_DIR}/discretePlanner.cpp
	    ${SRC_DIR}/gmmPlanner.cpp
)

add_library(${PROJECT_NAME} ${H_FILES} ${S_FILES} )
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${ARMADILLO_LIBRARIES} mlpack)