cmake_minimum_required(VERSION 3.5.1)
project(ugv_sdk VERSION 0.1.5)

find_program(CCACHE_PROGRAM ccache)
if(CCACHE_PROGRAM)
    message(STATUS "Found ccache")
    set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE "${CCACHE_PROGRAM}")
endif()

## Project Options
option(BUILD_TESTS ON)
option(STATIC_CHECK OFF)
# set(CMAKE_BUILD_TYPE Release)
# set(CMAKE_BUILD_TYPE Debug)

## Check if pkg is built with ROS catkin
if(CATKIN_DEVEL_PREFIX)
    message(STATUS "Build package with catkin")
    set(BUILD_WITHOUT_ROS OFF)
    find_package(catkin REQUIRED)
else()
    message(STATUS "Build package with cmake")
    set(BUILD_WITHOUT_ROS ON)    
endif()

## Generate symbols for IDE indexer
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(STATIC_CHECK)
    find_program(CPPCHECK cppcheck)
    if(CPPCHECK)
        message(STATUS "Found cppcheck")
        set(CMAKE_CXX_CPPCHECK cppcheck;--std=c++11;--enable=all)
    endif() 
endif()

## Additional cmake module path
set(USER_CMAKE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
list(APPEND CMAKE_MODULE_PATH "${USER_CMAKE_PATH}/modules")
list(APPEND CMAKE_PREFIX_PATH "/usr/lib/${CMAKE_SYSTEM_PROCESSOR}-linux-gnu/cmake")
list(APPEND CMAKE_PREFIX_PATH "/opt/weston_robot/lib/cmake")

## Set compiler to use c++ 11 features
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Chosse build type
set(default_build_type "Release")
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting build type to '${default_build_type}' as none was specified.")
  set(CMAKE_BUILD_TYPE "${default_build_type}" CACHE
      STRING "Choose the type of build." FORCE)
  # Set the possible values of build type for cmake-gui
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
    "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

if(BUILD_WITHOUT_ROS)
    ## Use GNUInstallDirs to install libraries into correct locations on all platforms.
    include(GNUInstallDirs)
    message(STATUS "Project will be installed to ${CMAKE_INSTALL_PREFIX} with 'make install'")

    ## Put all binary files into /bin and libraries into /lib
    set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR})
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR})
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/${CMAKE_INSTALL_BINDIR})
endif()

if(BUILD_WITHOUT_ROS)
    find_package(async_port REQUIRED)
else()
    find_package(catkin REQUIRED COMPONENTS async_port)
    include_directories(
        include
        ${catkin_INCLUDE_DIRS}
    )
endif()

# Build libraries
find_package(Threads REQUIRED)

add_library(${PROJECT_NAME} 
    # agx common
    src/agilex_base.cpp
    src/agx_msg_parser.c
    # robot support
    src/scout_base.cpp
    # src/hunter_base.cpp
    src/tracer_base.cpp
    # src/bunker_base.cpp
    src/ranger_base.cpp
)
if(BUILD_WITHOUT_ROS)
    target_link_libraries(${PROJECT_NAME} westonrobot::async_port Threads::Threads)
else()
    target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} Threads::Threads)
endif() 
target_include_directories(${PROJECT_NAME} PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>
    PRIVATE src)
    
# Build apps
if(BUILD_WITHOUT_ROS)
    # Check wether to build monitor tool
    set(BUILD_MONITOR ON)
    # Disable monitor if ncurses library is not found
    set(CURSES_NEED_NCURSES TRUE)
    find_package(Curses QUIET)
    if(BUILD_MONITOR AND NOT CURSES_FOUND)
        set(BUILD_MONITOR OFF)
        message(STATUS "Monitor app will not be built due to missing ncurses library, try: sudo apt install libncurses5-dev")
    endif()

    # add app source directory
    add_subdirectory(apps)
endif()

# Build tests
# if(PROJECT_NAME STREQUAL CMAKE_PROJECT_NAME AND BUILD_TESTS)
#     message(STATUS "Tests will be built")
#     enable_testing()
#     include(GoogleTest)
#     add_subdirectory(tests)
# else()
#     message(STATUS "Tests will not be built")
# endif()

## Installation configuration
if(BUILD_WITHOUT_ROS) # BUILD_WITHOUT_ROS

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
        message(STATUS " - To install ${p} components to ${_path}")
        unset(_path)
    endforeach()

    # targets to install
    install(TARGETS ${PROJECT_NAME}
        EXPORT ugv_sdkTargets
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include)

    install(DIRECTORY include/ugv_sdk
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

    # export target configuration 
    include(CMakePackageConfigHelpers)
    write_basic_package_version_file(
        ugv_sdkConfigVersion.cmake
        VERSION "${VERSION_MAJOR}.${VERSION_MINOR}.${VERSION_PATCH}"
        COMPATIBILITY SameMajorVersion)

    install(EXPORT ugv_sdkTargets
        FILE ugv_sdkTargets.cmake
        NAMESPACE westonrobot::
        DESTINATION lib/cmake/ugv_sdk)

    configure_file(cmake/ugv_sdkConfig.cmake.in ugv_sdkConfig.cmake @ONLY)
    install(FILES "${CMAKE_CURRENT_BINARY_DIR}/ugv_sdkConfig.cmake"
                "${CMAKE_CURRENT_BINARY_DIR}/ugv_sdkConfigVersion.cmake"
            DESTINATION lib/cmake/ugv_sdk)

    # Packaging support
    set(CPACK_PACKAGE_VENDOR "Weston Robot")
    set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Weston Robot Platform - SDK")
    set(CPACK_PACKAGE_VERSION_MAJOR ${PROJECT_VERSION_MAJOR})
    set(CPACK_PACKAGE_VERSION_MINOR ${PROJECT_VERSION_MINOR})
    set(CPACK_PACKAGE_VERSION_PATCH ${PROJECT_VERSION_PATCH})
    set(CPACK_PACKAGING_INSTALL_PREFIX "/opt/weston_robot")
    set(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_CURRENT_SOURCE_DIR}/LICENSE")
    # set(CPACK_RESOURCE_FILE_README "${CMAKE_CURRENT_SOURCE_DIR}/README.md")

    set(CPACK_GENERATOR "DEB")
    set(CPACK_DEBIAN_FILE_NAME DEB-DEFAULT)
    set(CPACK_DEBIAN_PACKAGE_MAINTAINER "Ruixiang Du (ruixiang.du@westonrobot.com)") 
    # set(CPACK_DEBIAN_PACKAGE_DEPENDS "libasio-dev")
    set(CPACK_SOURCE_IGNORE_FILES .git dist .*build.* /\\\\.DS_Store)
    include(CPack)

else() # BUILD_WITHOUT_ROS

    catkin_package(
        LIBRARIES ${PROJECT_NAME}
        INCLUDE_DIRS include 
         CATKIN_DEPENDS async_port
        #  DEPENDS system_lib
    )

    ## Add catkin install targets
    install(TARGETS ${PROJECT_NAME}
      RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
      LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
      ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

    install(DIRECTORY include/${PROJECT_NAME}
      DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})

endif() # BUILD_WITHOUT_ROS
