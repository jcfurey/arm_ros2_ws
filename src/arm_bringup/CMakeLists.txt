cmake_minimum_required(VERSION 3.20)
project(arm_bringup)
find_package(ament_cmake REQUIRED)
install(DIRECTORY launch worlds DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE)
install(PROGRAMS scripts/check_stack.py DESTINATION lib/${PROJECT_NAME} RENAME check_stack)
ament_package()
