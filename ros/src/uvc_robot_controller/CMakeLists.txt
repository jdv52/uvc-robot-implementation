cmake_minimum_required(VERSION 3.8)
project(uvc_robot_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  controller_interface
  generate_parameter_library
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  rcpputils
)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()
add_compile_definitions(RCLCPPUTILS_VERSION_MAJOR=${rclcpputils_VERSION_MAJOR})
add_compile_definitions(RCLCPPUTILS_VERSION_MAJOR=${rclcpputils_VERSION_MINOR})

add_library(uvc_controller SHARED
  src/uvc_controller.cpp
)
target_compile_features(uvc_controller PUBLIC cxx_std_17)
target_include_directories(uvc_controller PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/uvc_controller>
)
ament_target_dependencies(uvc_controller PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})
pluginlib_export_plugin_description_file(uvc_controller uvc_controller_plugin.xml)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

  # find_package(ament_lint_auto REQUIRED)
  # # the following line skips the linter which checks for copyrights
  # # comment the line when a copyright and license is added to all source files
  # set(ament_cmake_copyright_FOUND TRUE)
  # # the following line skips cpplint (only works in a git repo)
  # # comment the line when this package is in a git repo and when
  # # a copyright and license is added to all source files
  # set(ament_cmake_cpplint_FOUND TRUE)
  # ament_lint_auto_find_test_dependencies()

  add_definitions(-DTEST_FILES_DIRECTORY="${CMAKE_CURRENT_SOURCE_DIR}/test")
  ament_add_gmock(test_load_uvc_controller test/test_load_uvc_controller.cpp)
  ament_target_dependencies(test_load_uvc_controller
    controller_manager
    ros2_control_test_assets
  )
endif()

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS uvc_controller
  EXPORT export_uvc_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)


ament_package()
