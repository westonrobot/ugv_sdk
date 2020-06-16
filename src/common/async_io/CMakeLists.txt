# Dependency libraries
# find_package(LIB_NAME REQUIRED)

add_subdirectory(asio)

# Add libraries
set(ASYNC_IO_LIB_SRC
    src/async_serial.cpp
    src/async_can.cpp
    src/asyncio_utils.cpp
)
add_library(asyncio STATIC ${ASYNC_IO_LIB_SRC})
set_target_properties(asyncio PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_compile_definitions(asyncio PUBLIC "-DASIO_ENABLE_OLD_SERVICES -DASIO_HAS_POSIX_STREAM_DESCRIPTOR")
target_link_libraries(asyncio asio pthread)
target_include_directories(asyncio PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>
	PRIVATE src)

# install(TARGETS asyncio
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

# install(DIRECTORY include asio
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

# Add executables
if(BUILD_TESTS)
	add_subdirectory(tests)
endif()
