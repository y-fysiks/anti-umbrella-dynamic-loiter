cmake_minimum_required(VERSION 3.8)
project(dronepid)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  rclcpp_components
  geometry_msgs
  std_msgs
  apriltag_msgs
  apriltag
  apriltag_ros
  cv_bridge
)

## Find Dependencies
foreach(DEPENDENCY IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()

## Export Dependencies for downstream packages
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

## Add Libraries and Executables
function(add_node NODE_NAME)
  add_library(${NODE_NAME}_library
    src/${NODE_NAME}_node.cpp
    # Add other source files as needed
  )
  ament_target_dependencies(${NODE_NAME}_library ${THIS_PACKAGE_INCLUDE_DEPENDS})
  target_include_directories(${NODE_NAME}_library
      PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

  add_executable(${NODE_NAME} src/${NODE_NAME}.cpp)
  target_link_libraries(${NODE_NAME} ${NODE_NAME}_library)
  ament_target_dependencies(${NODE_NAME} ${THIS_PACKAGE_INCLUDE_DEPENDS})
  target_include_directories(${NODE_NAME}
      PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

  install(TARGETS ${NODE_NAME}_library
      ARCHIVE DESTINATION lib
      LIBRARY DESTINATION lib
      RUNTIME DESTINATION bin
      INCLUDES DESTINATION include/${PROJECT_NAME})

  install(TARGETS ${NODE_NAME}
      DESTINATION lib/${PROJECT_NAME})
endfunction()

add_node(dronepid)




## Install Headers
install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
