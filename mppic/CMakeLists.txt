cmake_minimum_required(VERSION 3.5)
project(mppic)

set(CMAKE_BUILD_TYPE Release) # temporary hack coz conan rise error with colcon build without setting build type



find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)

include(cmake/conan.cmake)
include(CTest)
include(Catch)

set(packages
  rclcpp 
  pluginlib 
  tf2 
  geometry_msgs 
  visualization_msgs
  nav_msgs 
  nav2_core 
  nav2_costmap_2d 
  nav2_util
  grid_map_msgs
  grid_map_ros
  xtensor
  xtensor-blas
)

foreach(pkg IN LISTS packages)
  find_package(${pkg} REQUIRED)
endforeach()

nav2_package()

add_subdirectory(src)

ament_export_libraries(mppic)
ament_export_dependencies(${packages})
ament_export_include_directories(include)
pluginlib_export_plugin_description_file(nav2_core mppic.xml)

add_subdirectory(test)

install(TARGETS mppic
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


install(DIRECTORY include/
  DESTINATION include/
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-error=unused-value -O0 -Wno-error=unused-variable -Wno-error=unused-parameter -Wno-error=narrowing")

ament_package()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-error=unused-value -O0 -Wno-error=unused-variable -Wno-error=unused-parameter -Wno-error=narrowing")