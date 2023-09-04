cmake_minimum_required(VERSION 3.0.2)
project(rs1)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs  # Add geometry_msgs since you use it in your code
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rs1
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Declare a C++ executable and specify the source file
add_executable(sprint2_sim1 src/sprint2_sim1.cpp)

# Add cmake target dependencies of the executable
add_dependencies(sprint2_sim1 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# Specify libraries to link against the executable target
target_link_libraries(sprint2_sim1
  ${catkin_LIBRARIES}
)
