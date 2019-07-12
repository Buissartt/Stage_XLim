cmake_minimum_required(VERSION 2.8.3)
add_compile_options(-std=c++11)

project(gui_leenby)
find_package(catkin REQUIRED COMPONENTS rviz roscpp image_transport cv_bridge sensor_msgs)
find_package(OpenCV REQUIRED)
catkin_package()
include_directories(${catkin_INCLUDE_DIRS} )
link_directories(${catkin_LIBRARY_DIRS})

## This setting causes Qt's "MOC" generation to happen automatically.
set(CMAKE_AUTOMOC ON)



  message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  message("Catkin library dirs : ${catkin_LIBRARY_DIRS}")
  
  find_package(Qt5 REQUIRED Core Widgets)
## make target_link_libraries(${QT_LIBRARIES}) pull in all required dependencies
  set(QT_LIBRARIES Qt5::Core Qt5::Widgets)

add_definitions(-DQT_NO_KEYWORDS)


file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp)
file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)
file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/gui_leenby/*.hpp)

qt5_add_resources(QT_RESOURCES_CPP ${QT_RESOURCES})
qt5_wrap_cpp(QT_MOC_HPP ${QT_MOC})

add_executable(gui_leenby ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_MOC_HPP} ${QT_FORMS_HPP})

target_link_libraries(gui_leenby ${QT_LIBRARIES} ${catkin_LIBRARIES})

