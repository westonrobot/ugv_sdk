# Dependency libraries
#find_package(LIBRARY_NAME REQUIRED)

add_subdirectory(googletest)

## Unit tests with GoogleTest
set(GTEST_SRC
    gtests/scout_can_protocol_test.cpp
)
add_executable(scout_sdk_test ${GTEST_SRC})
target_link_libraries(scout_sdk_test gtest gtest_main wrp_sdk)
target_include_directories(scout_sdk_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${GTEST_INCLUDE_DIRS}>
    $<INSTALL_INTERFACE:include>)