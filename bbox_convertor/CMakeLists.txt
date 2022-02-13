cmake_minimum_required(VERSION 2.8.3)
project(bbox_convertor)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  object_msgs
  roscpp
  tf 
  visualization_msgs  
  ackermann_msgs
  vision_msgs
)

find_package(OpenCV REQUIRED)

catkin_package()

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(bbox_convertor_node 
    src/bbox_convertor_node.cpp
)

add_dependencies(bbox_convertor_node
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(bbox_convertor_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
