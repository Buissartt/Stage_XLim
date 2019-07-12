cmake_minimum_required(VERSION 2.8.3)
project(premier_package)

find_package(catkin REQUIRED
COMPONENTS roscpp
)

include_directories(${catkin_INCLUDE_DIRS})

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES premier_package
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

add_executable(premier_noeud src/premier_noeud.cpp)
target_link_libraries(premier_noeud ${catkin_LIBRARIES})
