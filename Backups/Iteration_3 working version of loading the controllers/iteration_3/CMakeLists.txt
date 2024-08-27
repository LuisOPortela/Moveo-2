cmake_minimum_required(VERSION 3.22)
project(iteration_3)

find_package(ament_cmake REQUIRED)

ament_package()
install(DIRECTORY meshes DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})
