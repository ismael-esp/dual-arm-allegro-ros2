cmake_minimum_required(VERSION 3.5)
project(allegro_hand_parameters)

find_package(ament_cmake REQUIRED)

#############
## Install ##
#############

## Install all the parameter files
install(DIRECTORY .
  DESTINATION share/${PROJECT_NAME}/
  FILES_MATCHING PATTERN "*.yaml"
)
ament_package()
