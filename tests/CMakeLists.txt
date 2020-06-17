# Add executables
add_executable(test_aserial test_aserial.cpp)
target_link_libraries(test_aserial wrpsdk)

add_executable(test_aserial_comm test_aserial_comm.cpp)
target_link_libraries(test_aserial_comm wrpsdk)

add_executable(test_asio_can test_asio_can.cpp)
target_link_libraries(test_asio_can wrpsdk)

add_executable(test_acan test_acan.cpp)
target_link_libraries(test_acan wrpsdk)

# hunter
add_executable(test_hunter_base test_hunter_base.cpp)
target_link_libraries(test_hunter_base wrpsdk)

# scout
add_executable(test_scout_base test_scout_base.cpp)
target_link_libraries(test_scout_base wrpsdk)

# tracer
