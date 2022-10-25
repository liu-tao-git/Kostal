cmake_minimum_required(VERSION 3.5)
project(flexiv_description)

find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)

# generate robot control app urdf from xacro
file(GLOB xacro_files ${CMAKE_CURRENT_SOURCE_DIR}/urdf/*.urdf.xacro)
xacro_add_files(${xacro_files} TARGET media_files)

install(DIRECTORY urdf DESTINATION share/${PROJECT_NAME})
install(DIRECTORY meshes DESTINATION share/${PROJECT_NAME})

ament_package()
