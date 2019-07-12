cmake_minimum_required(VERSION 2.8.3)
add_compile_options(-std=c++11)
project(vocal_synthesis_subscriber)

find_package(catkin REQUIRED COMPONENTS roscpp)

include_directories(${catkin_INCLUDE_DIRS})

catkin_package()


add_executable(vocal_synthesis_subscriber src/main.cpp)
target_link_libraries(vocal_synthesis_subscriber ${catkin_LIBRARIES})
