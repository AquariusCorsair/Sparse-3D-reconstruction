cmake_minimum_required(VERSION 2.8.12)
project(S3D)
set (S3D_VERSION_MAJOR 1)
set (S3D_VERSION_MINOR 0)

find_package(OpenCV REQUIRED)
add_executable(exe main.cpp CamCalib.cpp)
target_link_libraries(exe ${OpenCV_LIBS})
set(RUNTIME_OUTPUT_DIRECTORY ./build)
