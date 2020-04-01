# Dependency libraries
find_package(Threads REQUIRED)

# Add libraries
add_library(stopwatch INTERFACE)
target_link_libraries(stopwatch INTERFACE ${CMAKE_THREAD_LIBS_INIT})
target_include_directories(stopwatch INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>)
