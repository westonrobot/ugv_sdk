cmake_minimum_required(VERSION 3.1.0)
project(wrp_sdk)

# generate symbols for IDE indexer
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/devel)

## Set compiler to use c++ 11 features
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Chosse build type
# set(CMAKE_BUILD_TYPE Release)
# set(CMAKE_BUILD_TYPE Debug)

## Optionally built modules: ON/OFF
set(BUILD_TESTS OFF)
set(BUILD_MONITOR ON)

#############################################################################

set(default_build_type "Release")
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting build type to '${default_build_type}' as none was specified.")
  set(CMAKE_BUILD_TYPE "${default_build_type}" CACHE
      STRING "Choose the type of build." FORCE)
  # Set the possible values of build type for cmake-gui
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
    "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

## Use GNUInstallDirs to install libraries into correct
# locations on all platforms.
include(GNUInstallDirs)
message(STATUS "Project will be installed to ${CMAKE_INSTALL_PREFIX}")

## Put all binary files into /bin and libraries into /lib
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/${CMAKE_INSTALL_BINDIR})

# Disable monitor if ncurses library is not found
set(CURSES_NEED_NCURSES TRUE)
find_package(Curses QUIET)

if(BUILD_MONITOR AND NOT CURSES_FOUND)
    set(BUILD_MONITOR OFF)
    message(STATUS "Monitor app will not be built due to missing ncurses library, try: sudo apt install libncurses5-dev")
endif()

# Add libraries
add_library(${PROJECT_NAME} 
    src/async_serial.cpp
    src/async_can.cpp
    src/asyncio_utils.cpp
    # agilex mobile platforms
    src/platforms/mobile_base.cpp
    src/platforms/hunter_base.cpp
    src/platforms/hunter_can_parser.c
    src/platforms/scout_base.cpp
    src/platforms/scout_can_parser.c
    src/platforms/scout_uart_parser.c
)
target_compile_definitions(${PROJECT_NAME} PUBLIC "-DASIO_ENABLE_OLD_SERVICES -DASIO_HAS_POSIX_STREAM_DESCRIPTOR")
target_link_libraries(${PROJECT_NAME} pthread)
target_include_directories(${PROJECT_NAME} PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>
	PRIVATE src)

# Add executables
if(BUILD_TESTS)
    add_subdirectory(tests)
    add_subdirectory(unit_tests)
endif()

# Offer the user the choice of overriding the installation directories
set(INSTALL_LIBDIR ${CMAKE_INSTALL_LIBDIR} CACHE PATH "Installation directory for libraries")
set(INSTALL_BINDIR ${CMAKE_INSTALL_BINDIR} CACHE PATH "Installation directory for executables")
set(INSTALL_INCLUDEDIR ${CMAKE_INSTALL_INCLUDEDIR} CACHE PATH "Installation directory for header files")
if(WIN32 AND NOT CYGWIN)
    set(DEF_INSTALL_CMAKEDIR CMake)
else()
    set(DEF_INSTALL_CMAKEDIR share/cmake/${PROJECT_NAME})
endif()
set(INSTALL_CMAKEDIR ${DEF_INSTALL_CMAKEDIR} CACHE PATH "Installation directory for CMake files")

# Report to user
foreach(p LIB BIN INCLUDE CMAKE)
    file(TO_NATIVE_PATH ${CMAKE_INSTALL_PREFIX}/${INSTALL_${p}DIR} _path)
    message(STATUS " -> To install ${p} components to ${_path}")
    unset(_path)
endforeach()

## Add install targets
# install(TARGETS ${PROJECT_NAME}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

# install(DIRECTORY asio/asio
#   DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})

# install(FILES asio/asio.hpp
#   DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})

# install(DIRECTORY scripts
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})