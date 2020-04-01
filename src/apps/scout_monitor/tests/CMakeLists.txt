# Add executables
add_executable(test_scout_monitor test_scout_monitor.cpp)
target_link_libraries(test_scout_monitor monitor)

add_executable(test_scout_monitor_virtual test_scout_monitor_virtual.cpp)
target_link_libraries(test_scout_monitor_virtual monitor)

# add_executable(test_ncurses test_ncurses.cpp)
# target_link_libraries(test_ncurses monitor)

# add_executable(test_ncolor test_ncolor.c)
# target_link_libraries(test_ncolor monitor)

# add_executable(test_ncolor2 test_ncolor2.cpp)
# target_link_libraries(test_ncolor2 monitor)