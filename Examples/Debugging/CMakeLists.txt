
include(FetchContent)
FetchContent_Declare(
  mcap_builder
  GIT_REPOSITORY https://github.com/olympus-robotics/mcap_builder.git
  GIT_TAG main
)
FetchContent_MakeAvailable(mcap_builder)

find_package(ament_cmake REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# If you want to deserialize sensor_msgs/msg/Image, for example:
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

add_executable(read_mcap read_mcap.cpp)
target_link_libraries(read_mcap
    ${PROJECT_NAME}  # The main library from the top-level
)
ament_target_dependencies(read_mcap
  rosbag2_cpp
  rosbag2_storage
  rclcpp
  sensor_msgs  # If needed
  cv_bridge
  OpenCV

  # ${PROJECT_SOURCE_DIR}/lib/libORB_SLAM3.so
)

add_executable(feature_extraction feature_extraction.cpp)
target_link_libraries(feature_extraction
  ${PROJECT_NAME}  # The main library from the top-level
)

install(TARGETS
  read_mcap
  feature_extraction
  DESTINATION lib/${PROJECT_NAME})
