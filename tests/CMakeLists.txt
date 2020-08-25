# Google tests
message(STATUS "Build unit tests with Google Test.")
add_subdirectory(googletest)

# reference: https://cliutils.gitlab.io/modern-cmake/chapters/testing/googletest.html
mark_as_advanced(
    BUILD_GMOCK BUILD_GTEST BUILD_SHARED_LIBS
    gmock_build_tests gtest_build_samples gtest_build_tests
    gtest_disable_pthreads gtest_force_shared_crt gtest_hide_internal_symbols
)

# add unit tests
set(GRAPH_TESTS
    unit_tests/scout_can_protocol_test.cpp
)
add_executable(gtest_all ${GRAPH_TESTS})
target_link_libraries(gtest_all gtest gmock gtest_main wrp_sdk)
