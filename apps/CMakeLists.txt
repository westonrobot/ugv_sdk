# Dependency libraries
#find_package(LIBRARY_NAME REQUIRED)

# tests
# add_executable(app_scout_demo scout_demo/scout_demo.cpp)
# target_link_libraries(app_scout_demo ugv_sdk)

# add_executable(app_scout_discharge scout_demo/scout_discharge.cpp)
# target_link_libraries(app_scout_discharge ugv_sdk)

# add_executable(app_hunter_demo hunter_demo/hunter_demo.cpp)
# target_link_libraries(app_hunter_demo ugv_sdk)

add_executable(app_tracer_demo tracer_demo/tracer_demo.cpp)
target_link_libraries(app_tracer_demo ugv_sdk)

add_executable(app_ranger_demo ranger_demo/ranger_demo.cpp)
target_link_libraries(app_ranger_demo ugv_sdk)

# if(BUILD_MONITOR)
#     add_subdirectory(scout_monitor)
# endif()
