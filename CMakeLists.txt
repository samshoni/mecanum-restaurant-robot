cmake_minimum_required(VERSION 3.8)
project(mecanum_bot)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  urdf
  models
  worlds
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/odom_tf_broadcaster.py
  scripts/clock_relay.py
  scripts/ign_tf_relay.py
  scripts/restaurant_gui.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
