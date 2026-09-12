cmake_minimum_required(VERSION 3.16)
project(rl_sim2sim_cpp)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(glfw3 REQUIRED)
find_package(mujoco REQUIRED)

find_library(ONNXRUNTIME_LIBRARY
  NAMES onnxruntime
  PATHS /opt/onnxruntime/lib
  REQUIRED
  NO_DEFAULT_PATH
)

include_directories(include)

add_executable(rl_sim2sim_node src/rl_sim2sim_node.cpp)

target_include_directories(rl_sim2sim_node PRIVATE
  /opt/onnxruntime/include
)

target_link_libraries(rl_sim2sim_node PRIVATE
  ${ONNXRUNTIME_LIBRARY}
  mujoco::mujoco
  glfw
)

set_property(TARGET rl_sim2sim_node PROPERTY CXX_STANDARD 17)
if(MSVC)
  target_compile_options(rl_sim2sim_node PRIVATE /EHa)
endif()

install(TARGETS rl_sim2sim_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
