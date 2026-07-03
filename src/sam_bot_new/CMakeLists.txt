cmake_minimum_required(VERSION 3.8)
project(sam_bot_new)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(dwb_core REQUIRED)
find_package(dwb_msgs REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(FUZZYLITE REQUIRED fuzzylite)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav_2d_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)

add_library(sam_bot_new SHARED
  src/fuzzy_dwa_critic.cpp
)

target_include_directories(sam_bot_new PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${FUZZYLITE_INCLUDE_DIRS}
)

target_link_libraries(sam_bot_new
  ${dwb_core_LIBRARIES}
  ${FUZZYLITE_LIBRARIES}
)

pluginlib_export_plugin_description_file(dwb_core sam_bot_new_plugins.xml)

install(
  TARGETS sam_bot_new
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(FILES sam_bot_new_plugins.xml DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(dwb_core nav2_core pluginlib rclcpp)

ament_package()
