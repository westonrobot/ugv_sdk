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