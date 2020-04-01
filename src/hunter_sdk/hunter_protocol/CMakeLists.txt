## Add libraries
set(HUNTER_PROTOCOL_SRC 
    src/hunter_can_parser.c
)
add_library(hunter_protocol STATIC ${HUNTER_PROTOCOL_SRC})
target_include_directories(hunter_protocol PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>	
    PRIVATE src)