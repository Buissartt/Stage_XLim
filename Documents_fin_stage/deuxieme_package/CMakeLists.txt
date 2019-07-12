cmake_minimum_required(VERSION 2.8.3)
project(deuxieme_package)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

include_directories(${catkin_INCLUDE_DIRS})

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES premier_package
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)


add_executable(deuxieme_noeud src/deuxieme_noeud.cpp)
target_link_libraries(deuxieme_noeud ${catkin_LIBRARIES})
