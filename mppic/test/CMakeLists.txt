if(mppic_BUILD_TESTING)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  -Wno-error=unused-value  -Wno-error=unused-variable -Wno-error=unused-parameter -Wno-error=narrowing -Wno-error=maybe-uninitialized")
  enable_testing()

  add_library(catch_main STATIC catch_main.cpp)
  ament_target_dependencies(catch_main rclcpp)
  target_link_libraries(catch_main Catch2::Catch2)


  if (mppic_BUILD_BENCHMARKS)
    target_compile_definitions(catch_main PUBLIC -DDO_BENCHMARKS)
  endif()

  if (mppic_TEST_DEBUG_INF)
    target_compile_definitions(catch_main PUBLIC -DTEST_DEBUG_INFO)
  endif()

  add_executable(optimizer-test optimizer-test.cpp)
  ament_target_dependencies(optimizer-test ${packages})

  target_link_libraries(optimizer-test tinyxml2::tinyxml2 xtensor xtensor-blas ${BLAS_LIBRARIES} ${LAPACK_LIBRARIES} catch_main mppic)
  target_include_directories(optimizer-test PRIVATE ../include)
  target_compile_options(optimizer-test PRIVATE -fconcepts)
  set_target_properties(optimizer-test
      PROPERTIES
          CXX_STANDARD 17
          CXX_STANDARD_REQUIRED YES
  )


  add_executable(geometry-test geometry-test.cpp)
  ament_target_dependencies(geometry-test ${packages})
  target_link_libraries(geometry-test xtensor catch_main mppic)

  target_include_directories(geometry-test PRIVATE ../include)
  target_compile_options(geometry-test PRIVATE -fconcepts)
  set_target_properties(geometry-test 
      PROPERTIES
          CXX_STANDARD 17
          CXX_STANDARD_REQUIRED YES
  )

  add_executable(grid-map-handler-test grid-map-handler-test.cpp)
  ament_target_dependencies(grid-map-handler-test ${packages})
  target_link_libraries(grid-map-handler-test catch_main mppic)
  target_include_directories(grid-map-handler-test PRIVATE ../include)
  target_compile_options(grid-map-handler-test PRIVATE -fconcepts)
  set_target_properties(grid-map-handler-test 
      PROPERTIES
          CXX_STANDARD 17
          CXX_STANDARD_REQUIRED YES
  )


add_custom_target(copy-test-makefile ALL DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/optimizer_test_config.xml)
add_custom_command(OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/optimizer_test_config.xml
                   COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/optimizer_test_config.xml
                                                    ${CMAKE_CURRENT_BINARY_DIR}/optimizer_test_config.xml
                   DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/optimizer_test_config.xml)

  catch_discover_tests(optimizer-test)
  catch_discover_tests(geometry-test)
  catch_discover_tests(grid-map-handler-test)

endif()
