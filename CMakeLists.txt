cmake_minimum_required(VERSION 3.27.3)

project(physics_engine)

find_package(GLUT REQUIRED)
find_package(SFML COMPONENTS system REQUIRED)

file(GLOB SRC_FILES src/*.cpp src/*.h)
add_executable(${PROJECT_NAME} ${SRC_FILES})

add_compile_definitions(_USE_MATH_DEFINES)
target_link_libraries(${PROJECT_NAME} GLUT::GLUT sfml-system)

add_custom_command(TARGET ${PROJECT_NAME} PRE_BUILD COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_SOURCE_DIR}/res/ $<TARGET_FILE_DIR:physics_engine>)
