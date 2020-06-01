## Add libraries
set(TRACER_BASE_SRC 
    src/tracer_base.cpp
)
add_library(tracerbase STATIC ${TRACER_BASE_SRC})
target_link_libraries(tracerbase asyncio tracer_protocol stopwatch)
target_include_directories(tracerbase PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${Boost_INCLUDE_DIRS}>
	$<INSTALL_INTERFACE:include>	
    PRIVATE src)

 ## Add executables
if(BUILD_TESTS)
    add_subdirectory(tests)
endif()