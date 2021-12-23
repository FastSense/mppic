cmake_minimum_required(VERSION 3.5)
project(mppic)

set(CMAKE_BUILD_TYPE Release) # temporary hack coz conan rise error with colcon build without setting build type
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-error=unused-but-set-variable -Wno-error=uninitialized -Wno-error=unused-value -Wno-error=unused-variable -Wno-error=unused-parameter -Wno-error=deprecated -Wno-error=narrowing -Wno-error=maybe-uninitialized -Wno-error=deprecated -Wno-error=pedantic -Wno-error=unused-but-set-variable -Wno-error=uninitialized -Wno-error=unused-value -Wno-error=parentheses -Wno-error=unused-variable -Wno-error=unused-parameter -Wno-error=narrowing -Wno-error=maybe-uninitialized")

find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)

option(mppic_BUILD_TESTING    "Build MPPI Controller with tests")
option(mppic_BUILD_BENCHMARKS "Include benchmarks into tests")
option(mppic_TEST_DEBUG_INF   "Show debug info during testing")

set(mppic_BUILD_TESTING ${BUILD_TESTING})
set(mppic_BUILD_BENCHMARKS ${BUILD_BENCHMARKS})
set(mppic_TEST_DEBUG_INF ${TEST_DEBUG_INFO})


include(CTest)

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
)

foreach(pkg IN LISTS packages)
  find_package(${pkg} REQUIRED)
endforeach()

add_definitions(-DHAVE_CBLAS=1)

find_package(BLAS REQUIRED)
find_package(LAPACK REQUIRED)

message(STATUS "BLAS VENDOR:    " ${BLA_VENDOR})
message(STATUS "BLAS LIBRARIES: " ${BLAS_LIBRARIES})

set(tinyxml2_BUILD_TESTING OFF)

add_subdirectory(external/catch2)
add_subdirectory(external/xtl)
add_subdirectory(external/xtensor)
add_subdirectory(external/xtensor-blas)
add_subdirectory(external/tinyxml2)

include(external/catch2/contrib/Catch.cmake)


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


ament_package()
