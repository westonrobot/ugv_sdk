cmake_minimum_required(VERSION 3.13.0)
project(asio)

# ascent library
add_library(asio INTERFACE)
target_compile_definitions(asio INTERFACE "-DASIO_STANDALONE")
target_include_directories(asio INTERFACE
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>)