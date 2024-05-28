pybind11_add_module(ugv_sdk_py
  src/ugv_sdk_py.cpp
  src/agilex_message.cpp
  src/scout_robot.cpp
  src/hunter_robot.cpp
  src/ranger_robot.cpp
)
target_link_libraries(ugv_sdk_py PRIVATE ugv_sdk)
target_include_directories(ugv_sdk_py PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  PRIVATE src)