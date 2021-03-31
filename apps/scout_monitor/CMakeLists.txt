# Dependency libraries
set(CURSES_NEED_NCURSES TRUE)
find_package(Curses REQUIRED)

# Add libraries
set(SCOUT_MONITOR_SRC
    src/nshapes.cpp
    src/ncolors.cpp
    src/scout_monitor.cpp
)
add_library(monitor STATIC ${SCOUT_MONITOR_SRC})
target_link_libraries(monitor ugv_sdk ${CURSES_LIBRARIES})
target_include_directories(monitor PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CURSES_INCLUDE_DIR}>
	$<INSTALL_INTERFACE:include>
	PRIVATE src)

add_subdirectory(app)

# Add executables
if(BUILD_TESTS)
	add_subdirectory(tests)
endif()