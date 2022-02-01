# if(mppic_BUILD_EXPERIMENTS)
#   # set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")


 
add_executable(experiments experiments_main.cpp)
ament_target_dependencies(experiments ${packages})

target_link_libraries(experiments tinyxml2 xtensor xtensor-blas ${BLAS_LIBRARIES} ${LAPACK_LIBRARIES} mppic)
target_include_directories(experiments PRIVATE ../include)
target_compile_options(experiments PRIVATE -fconcepts)
set_target_properties(experiments
    PROPERTIES
        CXX_STANDARD 17
        CXX_STANDARD_REQUIRED YES
)





