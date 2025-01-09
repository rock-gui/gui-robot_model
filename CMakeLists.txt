# CMakeLists.txt has to be located in the project folder and cmake has to be
# executed from 'project/build' with 'cmake ../'.
cmake_minimum_required(VERSION 3.16)
project(robot_model VERSION 1.0)
find_package(Rock)
rock_init()
rock_feature(NOCURDIR)

rock_standard_layout()
 
install(PROGRAMS bin/rock-roboviz
    DESTINATION bin)
    
