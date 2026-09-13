cmake_minimum_required(VERSION 3.8)
project(rl_sim2real_controller)

option(RL_SIM2REAL_TEST_POLICY
       "Print policy observations, actions, and joint targets" ON)
set(RL_TEST_LOG_PERIOD_MS 1000 CACHE STRING "Debug log period in milliseconds")

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)

set(REPO_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../../")
set(DEFAULT_JOINT_SCENE
    "${REPO_ROOT}/rl_mjlab/src/assets/robots/my_quadruped/mjcf/scene.xml")
set(DEFAULT_JOINT_ROBOT
    "${REPO_ROOT}/rl_mjlab/src/assets/robots/my_quadruped/mjcf/my_quadruped.xml")
set(DEFAULT_JOINT_GENERATOR "${REPO_ROOT}/tools/sync_default_joint.py")
set(DEFAULT_JOINT_INCLUDE_DIR "${CMAKE_CURRENT_BINARY_DIR}/generated_default_joint")
set(DEFAULT_JOINT_HEADER
    "${DEFAULT_JOINT_INCLUDE_DIR}/default_joint_constants.hpp")
add_custom_command(
  OUTPUT "${DEFAULT_JOINT_HEADER}"
  COMMAND "${Python3_EXECUTABLE}" "${DEFAULT_JOINT_GENERATOR}"
          --scene "${DEFAULT_JOINT_SCENE}" --cpp-output "${DEFAULT_JOINT_HEADER}"
  DEPENDS "${DEFAULT_JOINT_SCENE}" "${DEFAULT_JOINT_ROBOT}"
          "${DEFAULT_JOINT_GENERATOR}"
  VERBATIM
)
add_custom_target(generate_default_joint_constants
  DEPENDS "${DEFAULT_JOINT_HEADER}")

find_path(ONNXRUNTIME_INCLUDE_DIR onnxruntime_cxx_api.h
  PATHS /opt/onnxruntime/include REQUIRED NO_DEFAULT_PATH)
find_library(ONNXRUNTIME_LIBRARY onnxruntime
  PATHS /opt/onnxruntime/lib REQUIRED NO_DEFAULT_PATH)

add_library(${PROJECT_NAME} SHARED
  src/rl_sim2real_controller.cpp
)
add_dependencies(${PROJECT_NAME} generate_default_joint_constants)

target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
  ${ONNXRUNTIME_INCLUDE_DIR}
)
target_include_directories(${PROJECT_NAME} PRIVATE "${DEFAULT_JOINT_INCLUDE_DIR}")

ament_target_dependencies(${PROJECT_NAME}
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)
target_link_libraries(${PROJECT_NAME} ${ONNXRUNTIME_LIBRARY})
set_target_properties(${PROJECT_NAME} PROPERTIES
  BUILD_RPATH "/opt/onnxruntime/lib"
  INSTALL_RPATH "/opt/onnxruntime/lib"
)

if(RL_SIM2REAL_TEST_POLICY)
  target_compile_definitions(${PROJECT_NAME} PRIVATE
    RL_SIM2REAL_TEST_POLICY=1 RL_TEST_LOG_PERIOD_MS=${RL_TEST_LOG_PERIOD_MS})
endif()

pluginlib_export_plugin_description_file(
  controller_interface rl_sim2real_controller.xml
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

ament_package()
