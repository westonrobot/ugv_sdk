# Dependency libraries
#find_package(LIBRARY_NAME REQUIRED)

# tests
add_executable(test_hunter_base test_hunter_base.cpp)
target_link_libraries(test_hunter_base hunterbase)
