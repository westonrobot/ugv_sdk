## Add libraries
set(HUNTER_BASE_SRC 
    src/hunter_base.cpp
)
add_library(hunterbase STATIC ${HUNTER_BASE_SRC})
target_link_libraries(hunterbase asyncio hunter_protocol stopwatch)
target_include_directories(hunterbase PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>	
    PRIVATE src)

## Add executables
if(BUILD_TESTS)
    add_subdirectory(tests)
endif()