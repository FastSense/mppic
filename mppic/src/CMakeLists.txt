
add_library(mppic SHARED Controller.cpp PathHandler.cpp GridMapHandler.cpp)
target_include_directories(mppic PRIVATE ../include)
ament_target_dependencies(mppic
  ${packages}
)



target_link_libraries(mppic ${BLAS_LIBRARIES} ${LAPACK_LIBRARIES})
target_link_libraries(mppic xtensor)
target_link_libraries(mppic xtensor-blas)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-error=unused-value -Wno-error=unused-variable -Wno-error=unused-parameter -Wno-error=narrowing -Wno-error=maybe-uninitialized")
# prevent pluginlib from using boost
target_compile_definitions(mppic PRIVATE "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
target_compile_options(mppic PRIVATE -fconcepts )
set_target_properties(mppic
    PROPERTIES
        CXX_STANDARD 17
        CXX_STANDARD_REQUIRED YES
)

