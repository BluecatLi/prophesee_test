cmake_minimum_required(VERSION 3.0.2)
project(prophesee_test)

find_package(catkin REQUIRED COMPONENTS
        OpenCV
        roscpp
        sensor_msgs
        cv_bridge
        image_transport
        prophesee_event_msgs
        genmsg
        message_generation
)

find_package(MetavisionSDK COMPONENTS driver REQUIRED)

if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch)
endif()

catkin_package(
  INCLUDE_DIRS include/${PROJECT_NAME}/
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp sensor_msgs cv_bridge image_transport 
)

###########
## Build ##
###########

include_directories(
  include/${PROJECT_NAME}
  ${catkin_INCLUDE_DIRS}
  ${OPENCV_INCLUDE_DIRS}
)

# Intensity Variation
add_executable(prophesee_publisher
    src/prophesee_test.cpp

)

target_link_libraries(
    prophesee_publisher
    metavision_sdk_driver
    ${catkin_LIBRARIES}
)

add_dependencies(
    prophesee_publisher
    ${catkin_EXPORTED_TARGETS}
)
