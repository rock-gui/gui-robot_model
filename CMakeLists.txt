# CMakeLists.txt has to be located in the project folder and cmake has to be
# executed from 'project/build' with 'cmake ../'.
cmake_minimum_required(VERSION 2.6)
find_package(Rock)
rock_init(robot_model 1.0)
 
include(RockRuby)
rock_standard_layout()
 
install(PROGRAMS bin/rock-roboviz
    DESTINATION bin)
    
