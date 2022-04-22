cmake_minimum_required(VERSION 3.1)

get_filename_component(PROJECT_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
project(${PROJECT_NAME})


add_executable(${PROJECT_NAME}
        main.cpp
        viewer.h
        viewer.cpp
        imaging.cpp
        vector.h
        matrix.h
        ../myTest.cpp ../myTest.h)

target_include_directories(${PROJECT_NAME} PRIVATE ${EASY3D_INCLUDE_DIR})

target_compile_definitions(${PROJECT_NAME} PRIVATE GLEW_STATIC)

target_link_libraries(${PROJECT_NAME} easy3d_core easy3d_viewer easy3d_optimizer 3rd_cminpack)

