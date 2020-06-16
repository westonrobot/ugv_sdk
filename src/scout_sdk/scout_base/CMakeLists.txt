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

install(TARGETS scoutbase
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
  
install(DIRECTORY include
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

 ## Add executables
if(BUILD_TESTS)
    add_subdirectory(tests)
endif()