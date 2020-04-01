# Add executables
add_executable(test_aserial test_aserial.cpp)
target_link_libraries(test_aserial asyncio)

add_executable(test_aserial_comm test_aserial_comm.cpp)
target_link_libraries(test_aserial_comm asyncio)

add_executable(test_asio_can test_asio_can.cpp)
target_link_libraries(test_asio_can asio pthread)

add_executable(test_acan test_acan.cpp)
target_link_libraries(test_acan asyncio)