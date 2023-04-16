cmake_minimum_required(VERSION 3.5)
project(ardupilot_gz_description)

# --------------------------------------------------------------------------- #
# Find dependencies.
find_package(ament_cmake REQUIRED)

# --------------------------------------------------------------------------- #
#  Install.

install(
  DIRECTORY
    models/
  DESTINATION share/${PROJECT_NAME}/models
)

# --------------------------------------------------------------------------- #
#  Build tests.

if(BUILD_TESTING)
  # Override default flake8 configuration.
  set(ament_cmake_flake8_CONFIG_FILE ${CMAKE_SOURCE_DIR}/.flake8)

  # Add linters.
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# --------------------------------------------------------------------------- #
#  Environment hooks.

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.sh.in")

# --------------------------------------------------------------------------- #
#  Call last.

ament_package()
