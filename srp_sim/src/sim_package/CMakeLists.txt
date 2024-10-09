cmake_minimum_required(VERSION 3.8)
project(sim_package)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)


find_package(ignition-cmake2 REQUIRED)
find_package(ignition-plugin1 REQUIRED COMPONENTS register)
set(GZ_PLUGIN_VER ${ignition-plugin1_VERSION_MAJOR})
find_package(ignition-common4 REQUIRED COMPONENTS profiler)
set(GZ_COMMON_VER ${ignition-common4_VERSION_MAJOR})
find_package(ignition-gazebo6 REQUIRED)
set(GZ_SIM_VER ${ignition-gazebo6_VERSION_MAJOR})
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
include_directories(include)
install(DIRECTORY models/
    DESTINATION share/${PROJECT_NAME}/models
)

install(DIRECTORY worlds/
    DESTINATION share/${PROJECT_NAME}/worlds
)
install(DIRECTORY launch/
	DESTINATION share/${PROJECT_NAME}/launch
)
install(DIRECTORY config/
	DESTINATION share/${PROJECT_NAME}/config
)

add_library(BasicSystem
  SHARED
  src/BasicSystem.cc
)
target_include_directories(
  BasicSystem PRIVATE include
)


target_link_libraries(BasicSystem PRIVATE
  ignition-gazebo${GZ_SIM_VER}::ignition-gazebo${GZ_SIM_VER})

install(
  TARGETS BasicSystem 
  DESTINATION lib/${PROJECT_NAME}
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

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/sim_package.dsv.in")
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/sim_package.sh.in")

ament_package()
