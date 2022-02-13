cmake_minimum_required(VERSION 2.8.3)
project(person_follower_sensor_fusion)

add_compile_options(-std=c++11)

find_package(OpenCV REQUIRED )

find_package(catkin REQUIRED COMPONENTS
  object_msgs
  roscpp
  tf
  depth_image_proc
  image_transport
  cv_bridge
  image_geometry
  visualization_msgs
  angles
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

add_executable(person_follower_sensor_fusion_node 
    src/person_follower_sensor_fusion_node.cpp
    src/person_follower_sensor_fusion.cpp
)

add_dependencies(person_follower_sensor_fusion_node
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(person_follower_sensor_fusion_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
