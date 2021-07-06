cmake_minimum_required(VERSION 3.10.2)
project(sdk_sample)

find_package(ugv_sdk REQUIRED)
find_package(Threads REQUIRED)

add_executable(app_tracer_demo tracer_demo.cpp)
target_link_libraries(app_tracer_demo PRIVATE ugv_sdk Threads::Threads)