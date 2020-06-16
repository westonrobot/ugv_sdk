## Add libraries
set(SCOUT_PROTOCOL_SRC 
    src/scout_can_parser.c
    src/scout_uart_parser.c
)
add_library(scout_protocol STATIC ${SCOUT_PROTOCOL_SRC})
set_target_properties(scout_protocol PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(scout_protocol PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${Boost_INCLUDE_DIRS}>
	$<INSTALL_INTERFACE:include>	
    PRIVATE src)

install(TARGETS scout_protocol
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
  
install(DIRECTORY include
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS scout_protocol
    # EXPORT MyLibTargets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    INCLUDES DESTINATION include)