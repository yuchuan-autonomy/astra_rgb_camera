cmake_minimum_required(VERSION 3.8)
project(astra_rgb_camera)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc videoio)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(device_discovery src/device_discovery.cpp)
target_compile_features(device_discovery PUBLIC cxx_std_17)
target_include_directories(device_discovery PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(rgb_camera_node src/rgb_camera_node.cpp)
target_compile_features(rgb_camera_node PUBLIC cxx_std_17)
target_include_directories(rgb_camera_node PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(rgb_camera_node device_discovery ${OpenCV_LIBS})
ament_target_dependencies(rgb_camera_node rclcpp sensor_msgs)

install(TARGETS device_discovery rgb_camera_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_device_discovery test/test_device_discovery.cpp)
  target_compile_features(test_device_discovery PUBLIC cxx_std_17)
  target_link_libraries(test_device_discovery device_discovery)
endif()

ament_export_include_directories(include)
ament_export_libraries(device_discovery)
ament_package()
