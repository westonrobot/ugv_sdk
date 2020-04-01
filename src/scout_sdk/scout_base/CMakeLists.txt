## Add libraries
set(SCOUT_BASE_SRC 
    src/scout_base.cpp
)
add_library(scoutbase STATIC ${SCOUT_BASE_SRC})
target_link_libraries(scoutbase asyncio scout_protocol stopwatch)
target_include_directories(scoutbase PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${Boost_INCLUDE_DIRS}>
	$<INSTALL_INTERFACE:include>	
    PRIVATE src)

 ## Add executables
if(BUILD_TESTS)
    add_subdirectory(tests)
endif()