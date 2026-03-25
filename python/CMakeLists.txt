# Python bindings for rpl_rio

# Find Python and nanobind
find_package(Python REQUIRED COMPONENTS Interpreter Development)
execute_process(
  COMMAND "${Python_EXECUTABLE}" -m nanobind --cmake_dir
  OUTPUT_STRIP_TRAILING_WHITESPACE OUTPUT_VARIABLE NB_DIR
  RESULT_VARIABLE NB_RESULT)
message(STATUS "nanobind cmake dir: ${NB_DIR}")
list(APPEND CMAKE_PREFIX_PATH "${NB_DIR}")
find_package(nanobind CONFIG REQUIRED)

# Create nanobind module
nanobind_add_module(_core bindings.cpp)
target_link_libraries(_core PRIVATE rio)
target_include_directories(_core PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../src>
)

# Install nanobind extension into the Python package
install(TARGETS _core
  LIBRARY DESTINATION "${PYTHON_INSTALL_DIR}/rpl_rio"
)
