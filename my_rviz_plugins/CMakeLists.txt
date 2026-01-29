cmake_minimum_required(VERSION 3.0.2)
project(my_rviz_plugins)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rviz
  pluginlib
  std_msgs
  geometry_msgs
  std_srvs
  actionlib
  mbf_msgs
  message_generation
)


# ===== Services =====
add_service_files(
  FILES
  xju_task.srv
  keepOutZone.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

# ===== Qt =====
find_package(Qt5 REQUIRED COMPONENTS Widgets)

# 关键：启用 Qt MOC（catkin下 Qt5 常用）
set(CMAKE_AUTOMOC OFF)

# ===== catkin package export（只写一次！）=====
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rviz pluginlib std_msgs geometry_msgs message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS}
)

# ===== moc =====
qt5_wrap_cpp(MOC_FILES
  include/my_rviz_plugins/my_panel.h
)

add_library(${PROJECT_NAME}
  src/my_panel.cpp
  ${MOC_FILES}
)

# 确保 srv 先生成，再编译库（非常关键）
add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  Qt5::Widgets
)

install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(FILES plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
