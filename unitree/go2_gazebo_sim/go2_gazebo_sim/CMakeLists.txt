cmake_minimum_required(VERSION 3.8)
project(go2_gazebo_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)
install(
  DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  go2_gazebo_sim/go2_remapping.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME go2_remapping_node
)

install(PROGRAMS
  go2_gazebo_sim/go2_sport.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME go2_sport_node
)

install(PROGRAMS
  go2_gazebo_sim/go2_lowstate.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME go2_lowstate_node
)

install(PROGRAMS
  go2_gazebo_sim/intel435_depth.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME intel435_depth_node
)

install(PROGRAMS
  go2_gazebo_sim/joy_robot_switcher.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME joy_robot_switcher_node
)

ament_package()
