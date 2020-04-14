## Add libraries
set(TRACER_PROTOCOL_SRC 
    src/tracer_can_parser.c
)
add_library(tracer_protocol STATIC ${TRACER_PROTOCOL_SRC})
target_include_directories(tracer_protocol PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>	
    PRIVATE src)