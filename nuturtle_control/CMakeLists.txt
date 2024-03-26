cmake_minimum_required(VERSION 3.8)
project(nuturtle_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# set(turtlelib_DIR turtlelib/cmake)


# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
# find_package(nuturtle_description REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nuturtlebot_msgs REQUIRED)
find_package(turtlelib REQUIRED)
find_package(nav_msgs REQUIRED)
# find_package(catch_ros2 REQUIRED)


set(srv_files
  "srv/InitialPose.srv"
  "srv/Control.srv"
)
rosidl_generate_interfaces(${PROJECT_NAME}
  ${srv_files}
)


add_executable(turtle_control_node src/turtle_control.cpp)
ament_target_dependencies(
  turtle_control_node
  turtlelib
  rclcpp
  std_msgs
  std_srvs
  # nuturtle_description
  tf2
  tf2_ros
  geometry_msgs
  rosidl_default_generators
  visualization_msgs
  nuturtlebot_msgs
  nav_msgs)

add_executable(odometry_node src/odometry.cpp)
ament_target_dependencies(
  odometry_node
  turtlelib
  rclcpp
  std_msgs
  std_srvs
  # nuturtle_description
  tf2
  tf2_ros
  geometry_msgs
  rosidl_default_generators
  visualization_msgs
  nuturtlebot_msgs
  nav_msgs)

add_executable(circle_node src/circle.cpp)
ament_target_dependencies(
    circle_node
    turtlelib
    rclcpp
    std_msgs
    std_srvs
    # nuturtle_description
    tf2
    tf2_ros
    geometry_msgs
    rosidl_default_generators
    visualization_msgs
    nuturtlebot_msgs
    nav_msgs)


rosidl_target_interfaces(odometry_node ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(odometry_node "${cpp_typesupport_target}")

rosidl_target_interfaces(circle_node ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(circle_node "${cpp_typesupport_target}")

target_link_libraries(turtle_control_node turtlelib::turtlelib "${cpp_typesupport_target}")
target_link_libraries(odometry_node turtlelib::turtlelib "${cpp_typesupport_target}")
target_link_libraries(circle_node turtlelib::turtlelib "${cpp_typesupport_target}")


install(TARGETS
  turtle_control_node
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  odometry_node
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  circle_node
  DESTINATION lib/${PROJECT_NAME})

# Install launch files.
install(DIRECTORY
  srv launch config
  DESTINATION share/${PROJECT_NAME}/
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

if(BUILD_TESTING)
  include(CTest)

  find_package(catch_ros2 REQUIRED)

  ###########################
  ## INTEGRATION TEST NODE ##
  ###########################
  # This is the node in which integration tests occur
  add_executable(turtle_control_test_node
    test/turtle_control_test.cpp
  )
  add_executable(odometry_test_node
  test/turtle_odom_test.cpp
  )
  add_executable(turtle_circle_test_node
  test/turtle_circle_test.cpp
  )

  # The link libraries call links this node with catch_ros2::catch_ros2_with_node_main
  # to get the default integration test node main function
  target_link_libraries(turtle_control_test_node
    catch_ros2::catch_ros2_with_node_main
  )
  target_link_libraries(odometry_test_node
  catch_ros2::catch_ros2_with_node_main
  )
  target_link_libraries(turtle_circle_test_node
  catch_ros2::catch_ros2_with_node_main
  )

  ament_target_dependencies(
    turtle_control_test_node
    turtlelib
    rclcpp
    std_msgs
    std_srvs
    # nuturtle_description
    tf2
    tf2_ros
    geometry_msgs
    rosidl_default_generators
    visualization_msgs
    nuturtlebot_msgs
    nav_msgs
    # catch_ros2
  )
  ament_target_dependencies(
    odometry_test_node
    turtlelib
    rclcpp
    std_msgs
    std_srvs
    # nuturtle_description
    tf2
    tf2_ros
    geometry_msgs
    rosidl_default_generators
    visualization_msgs
    nuturtlebot_msgs
    nav_msgs
    # catch_ros2
  )
  ament_target_dependencies(
    turtle_circle_test_node
    turtlelib
    rclcpp
    std_msgs
    std_srvs
    # nuturtle_description
    tf2
    tf2_ros
    geometry_msgs
    rosidl_default_generators
    visualization_msgs
    nuturtlebot_msgs
    nav_msgs
    # catch_ros2
  )

  rosidl_target_interfaces(odometry_test_node ${PROJECT_NAME} "rosidl_typesupport_cpp")
  target_link_libraries(odometry_test_node "${cpp_typesupport_target}")

  rosidl_target_interfaces(turtle_circle_test_node ${PROJECT_NAME} "rosidl_typesupport_cpp")
  target_link_libraries(turtle_circle_test_node "${cpp_typesupport_target}")

  target_link_libraries(turtle_control_test_node turtlelib::turtlelib "${cpp_typesupport_target}")
  target_link_libraries(odometry_test_node turtlelib::turtlelib "${cpp_typesupport_target}")
  target_link_libraries(turtle_circle_test_node turtlelib::turtlelib "${cpp_typesupport_target}")


  install(TARGETS
    turtle_control_test_node
    odometry_test_node
    turtle_circle_test_node
    DESTINATION lib/${PROJECT_NAME}
  )

  #################
  ## LAUNCH FILE ##
  #################
  # This is the launch file that will be used to run the integration test
  install(FILES
    test/turtle_control_test.launch.xml
    test/turtle_odom_test.launch.xml
    test/turtle_circle_test.launch.xml
    DESTINATION share/${PROJECT_NAME}
  )

  ######################
  ## INTEGRATION TEST ##
  ######################
  # This CMake function allows the integration test to be run
  # when running "colcon test".
  catch_ros2_add_integration_test(TurtleControl_TestXML
    LAUNCH_FILE turtle_control_test.launch.xml  # example in XML
  )

  catch_ros2_add_integration_test(Odometry_TestXML
    LAUNCH_FILE turtle_odom_test.launch.xml  # example in XML
  )

  catch_ros2_add_integration_test(TurtleCircle_TestXML
    LAUNCH_FILE turtle_circle_test.launch.xml  # example in XML
  )

endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
