cmake_minimum_required(VERSION 3.8)
project(path_controller)

# Поддержка Qt MOC/UIC
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Зависимости
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Qt5Widgets REQUIRED)
find_package(ament_index_cpp REQUIRED)

# Include --------------------------------------------------------------------------------
include_directories(
  include
)

# Библиотеки утилит ---------------------------------------------------------------------
add_library(landmark_utils
  src/landmark_utils.cpp
)
ament_target_dependencies(landmark_utils
  rclcpp
  std_msgs
)

add_library(snail_utils
  src/snail_utils.cpp
)

# Библиотеки UI --------------------------------------------------------------------------
add_library(map_widget
  src/map_widget.cpp
  include/path_controller/map_widget.hpp
)
find_package(Qt5Widgets REQUIRED)
target_link_libraries(map_widget Qt5::Widgets)
ament_target_dependencies(map_widget)

add_library(map_visualizer
  src/map_visualizer.cpp
  src/gui.cpp
)
target_link_libraries(map_visualizer
  map_widget
)
ament_target_dependencies(map_visualizer
  rclcpp
  nav_msgs
  geometry_msgs
  std_msgs
  ament_index_cpp
)

# Исполняемые ----------------------------------------------------------------------------
## Визуализатор (Qt + ROS)
add_executable(map_visualizer_app
  src/map_visualizer_node.cpp
  src/main.cpp
)
target_link_libraries(map_visualizer_app
  map_visualizer
  landmark_utils
  snail_utils
)
ament_target_dependencies(map_visualizer_app
  rclcpp
  nav_msgs
  geometry_msgs
  std_msgs
  ament_index_cpp
  Qt5Widgets
)

## Прочие узлы
add_executable(map_loader         src/map_loader.cpp)
add_executable(robot_controller   src/robot_controller.cpp)
add_executable(path_planner       src/path_planner.cpp)
add_executable(localization_node  src/localization_node.cpp)
add_executable(goal_handler       src/goal_handler.cpp)

ament_target_dependencies(map_loader
  rclcpp
  nav_msgs
  geometry_msgs
  OpenCV
  ament_index_cpp
)
ament_target_dependencies(robot_controller
  rclcpp
  geometry_msgs
  std_msgs
  ament_index_cpp
)
ament_target_dependencies(path_planner
  rclcpp
  nav_msgs
  geometry_msgs
)
ament_target_dependencies(localization_node
  rclcpp
  std_msgs
  geometry_msgs
  ament_index_cpp
)
ament_target_dependencies(goal_handler
  rclcpp
  geometry_msgs
)

# Установка -------------------------------------------------------------------------------
install(
  TARGETS
    map_loader
    robot_controller
    path_planner
    localization_node
    goal_handler
    map_visualizer_app
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY assets/
  DESTINATION share/${PROJECT_NAME}/assets
)

# Тестирование ---------------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
