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
set(PYBIND_OUTPUT_LIBDIR ${PROJECT_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR} CACHE PATH "Output directory for pybind libraries")
message(STATUS " - To install pybind components to ${PYBIND_OUTPUT_LIBDIR}")
set_target_properties(ugv_sdk_py PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY ${PYBIND_OUTPUT_LIBDIR}
)