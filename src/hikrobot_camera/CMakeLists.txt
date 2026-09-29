cmake_minimum_required(VERSION 3.8)
project(hikrobot_camera)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(camera_node
  src/main.cpp
  src/camera_node.cpp
)
target_include_directories(camera_node PRIVATE
        include
        /opt/MVS/include
)
target_compile_features(camera_node PRIVATE cxx_std_17)

ament_target_dependencies(camera_node
        rclcpp
        sensor_msgs
)

target_link_libraries(camera_node
        /opt/MVS/lib/64/libMvCameraControl.so
)
target_link_libraries(camera_node
        /opt/MVS/lib/64/libMvCameraControl.so
        ${OpenCV_LIBS}
)
target_include_directories(camera_node PRIVATE
        include
        /opt/MVS/include
        ${OpenCV_INCLUDE_DIRS}
)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(camera_node PRIVATE -Wall -Wextra -Wpedantic)
endif()

install(TARGETS camera_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()
