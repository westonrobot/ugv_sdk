# Add executables
add_executable(test_aserial test_aserial.cpp)
target_link_libraries(test_aserial wrp_sdk)

add_executable(test_aserial_comm test_aserial_comm.cpp)
target_link_libraries(test_aserial_comm wrp_sdk)

add_executable(test_asio_can test_asio_can.cpp)
target_link_libraries(test_asio_can wrp_sdk)

add_executable(test_acan test_acan.cpp)
target_link_libraries(test_acan wrp_sdk)

# hunter
add_executable(test_hunter_base test_hunter_base.cpp)
target_link_libraries(test_hunter_base wrp_sdk)

# scout
add_executable(test_scout_base test_scout_base.cpp)
target_link_libraries(test_scout_base wrp_sdk)

# tracer
