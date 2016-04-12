cmake_minimum_required(VERSION 2.8.3)
project(exploration_planner)

find_package(catkin REQUIRED roscpp robot_base_planners optitrack_rviz message_generation statistics_ml peg_sensor)

set(CMAKE_CXX_FLAGS "-O2 -O3 -std=c++11 -mtune=native -fPIC -Wall -Wno-unused-function -Wno-deprecated-declarations")

find_package(Armadillo REQUIRED)

add_service_files(
   FILES
   String_cmd.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp message_generation optitrack_rviz statistics_ml robot_base_planners peg_sensor
)

set(INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)
set(SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)


include_directories(include ${catkin_INCLUDE_DIRS} ${INCLUDE_DIR})


set(H_FILES ${INCLUDE_DIR}/exploration_planner/baseExploration.h
	    ${INCLUDE_DIR}/exploration_planner/singleExploration.h
	    ${INCLUDE_DIR}/exploration_planner/searchPlanner.h
            ${INCLUDE_DIR}/exploration_planner/simple_exploration.h
            ${INCLUDE_DIR}/exploration_planner/belief_gmm_planner.h


)
set(S_FILES ${SRC_DIR}/searchPlanner.cpp
	    ${SRC_DIR}/baseExploration.cpp
            ${SRC_DIR}/simple_exploration.cpp
            ${SRC_DIR}/belief_gmm_planner.cpp
)


add_library(${PROJECT_NAME} ${H_FILES} ${S_FILES})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${ARMADILLO_LIBRARIES} mlpack )


add_executable(test_exploration ${SRC_DIR}/test/test_planner.cpp)
target_link_libraries(test_exploration ${catkin_LIBRARIES} ${ARMADILLO_LIBRARIES} ${PROJECT_NAME} mlpack  )
