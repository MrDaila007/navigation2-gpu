cmake_minimum_required(VERSION 3.20)
project(nav2_mppi_controller)

find_package(ament_cmake REQUIRED)
find_package(angles REQUIRED)
find_package(backward_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav2_ros_common REQUIRED)
find_package(Eigen3 REQUIRED)

nav2_package()

include(CheckCXXCompilerFlag)

check_cxx_compiler_flag("-mfma" COMPILER_SUPPORTS_FMA)

if(COMPILER_SUPPORTS_FMA)
  # Keep the host flag off CUDA compilation. nvcc rejects -mfma as its own option.
  add_compile_options($<$<COMPILE_LANGUAGE:CXX>:-mfma>)
endif()

# If building one the same hardware to be deployed on, try `-march=native`!

add_library(mppi_controller SHARED
  src/controller.cpp
  src/critic_manager.cpp
  src/noise_generator.cpp
  src/optimizer.cpp
  src/parameters_handler.cpp
  src/trajectory_visualizer.cpp
)
target_compile_options(mppi_controller PUBLIC -O3)
target_include_directories(mppi_controller
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(mppi_controller PUBLIC
  Eigen3::Eigen
  angles::angles
  geometry_msgs::geometry_msgs
  nav2_core::nav2_core
  nav2_ros_common::nav2_ros_common
  nav2_costmap_2d::layers
  nav2_costmap_2d::nav2_costmap_2d_core
  nav2_msgs::nav2_msgs
  nav_msgs::nav_msgs
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  std_msgs::std_msgs
  tf2::tf2
  tf2_geometry_msgs::tf2_geometry_msgs
  visualization_msgs::visualization_msgs
)

target_link_libraries(mppi_controller PRIVATE
  pluginlib::pluginlib
)

add_library(mppi_motion_models SHARED)
target_sources(mppi_motion_models PRIVATE
  src/motion_models.cpp
)
target_compile_options(mppi_motion_models PUBLIC -O3)
target_include_directories(mppi_motion_models
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(mppi_motion_models PUBLIC
  Eigen3::Eigen
  mppi_controller
  pluginlib::pluginlib
)

option(NAV2_MPPI_CUDA "Build MPPI CostCritic CUDA scoring" OFF)

# OFF keeps the stub. ON replaces it with the device kernel.
set(MPPI_COST_CRITIC_CUDA_SRC src/critics/cost_critic_cuda_stub.cpp)
if(NAV2_MPPI_CUDA)
  # nav2_package() adds -Werror for every language. nvcc treats the following
  # -MD as the argument of -Werror, so keep those host flags on C++ only.
  get_directory_property(_nav2_mppi_compile_options COMPILE_OPTIONS)
  set(_nav2_mppi_scoped_options)
  foreach(_opt IN LISTS _nav2_mppi_compile_options)
    if(_opt MATCHES "COMPILE_LANGUAGE")
      list(APPEND _nav2_mppi_scoped_options "${_opt}")
    else()
      list(APPEND _nav2_mppi_scoped_options "$<$<COMPILE_LANGUAGE:CXX>:${_opt}>")
    endif()
  endforeach()
  set_property(DIRECTORY PROPERTY COMPILE_OPTIONS ${_nav2_mppi_scoped_options})

  if(NOT CMAKE_CUDA_ARCHITECTURES)
    execute_process(
      COMMAND nvidia-smi --query-gpu=compute_cap --format=csv,noheader
      OUTPUT_VARIABLE NAV2_MPPI_GPU_COMPUTE_CAP
      OUTPUT_STRIP_TRAILING_WHITESPACE
      RESULT_VARIABLE NAV2_MPPI_GPU_COMPUTE_CAP_RESULT
    )
    if(NOT NAV2_MPPI_GPU_COMPUTE_CAP_RESULT EQUAL 0 OR NAV2_MPPI_GPU_COMPUTE_CAP STREQUAL "")
      message(FATAL_ERROR
        "CMAKE_CUDA_ARCHITECTURES is empty and nvidia-smi --query-gpu=compute_cap failed")
    endif()
    string(REGEX REPLACE "[\r\n].*" "" NAV2_MPPI_GPU_COMPUTE_CAP "${NAV2_MPPI_GPU_COMPUTE_CAP}")
    string(STRIP "${NAV2_MPPI_GPU_COMPUTE_CAP}" NAV2_MPPI_GPU_COMPUTE_CAP)
    string(REPLACE "." "" NAV2_MPPI_GPU_COMPUTE_CAP "${NAV2_MPPI_GPU_COMPUTE_CAP}")
    set(CMAKE_CUDA_ARCHITECTURES "${NAV2_MPPI_GPU_COMPUTE_CAP}")
  endif()
  message(STATUS "CMAKE_CUDA_ARCHITECTURES=${CMAKE_CUDA_ARCHITECTURES}")
  enable_language(CUDA)
  find_package(CUDAToolkit REQUIRED)
  set(CMAKE_CUDA_STANDARD 17)
  set(MPPI_COST_CRITIC_CUDA_SRC src/critics/cost_critic_cuda.cu)
endif()

add_library(mppi_critics SHARED
  src/critics/constraint_critic.cpp
  src/critics/cost_critic.cpp
  ${MPPI_COST_CRITIC_CUDA_SRC}
  src/critics/goal_critic.cpp
  src/critics/goal_angle_critic.cpp
  src/critics/mecanum_critic.cpp
  src/critics/obstacles_critic.cpp
  src/critics/omni_constraint_critic.cpp
  src/critics/path_align_critic.cpp
  src/critics/path_angle_critic.cpp
  src/critics/path_follow_critic.cpp
  src/critics/prefer_forward_critic.cpp
  src/critics/twirling_critic.cpp
  src/critics/velocity_deadband_critic.cpp
)
if(CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    # Apple Clang: use C++20 and optimization, omit -fconcepts
    target_compile_features(mppi_critics PUBLIC cxx_std_20)
    target_compile_options(mppi_critics PUBLIC $<$<COMPILE_LANGUAGE:CXX>:-O3>)
else()
    target_compile_options(mppi_critics PUBLIC
      $<$<COMPILE_LANGUAGE:CXX>:-fconcepts>
      $<$<COMPILE_LANGUAGE:CXX>:-O3>)
endif()
target_include_directories(mppi_critics
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(mppi_critics PUBLIC
  Eigen3::Eigen
  angles::angles
  geometry_msgs::geometry_msgs
  nav2_core::nav2_core
  nav2_ros_common::nav2_ros_common
  nav2_costmap_2d::layers
  nav2_costmap_2d::nav2_costmap_2d_core
  nav_msgs::nav_msgs
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  std_msgs::std_msgs
  tf2::tf2
  tf2_geometry_msgs::tf2_geometry_msgs
  visualization_msgs::visualization_msgs
)
target_link_libraries(mppi_critics PRIVATE
  pluginlib::pluginlib
)
if(NAV2_MPPI_CUDA)
  target_compile_definitions(mppi_critics PRIVATE NAV2_MPPI_HAS_CUDA=1)
  target_link_libraries(mppi_critics PRIVATE CUDA::cudart)
endif()

add_library(mppi_trajectory_validators SHARED
  src/trajectory_validators/optimal_trajectory_validator.cpp
)
target_compile_options(mppi_trajectory_validators PUBLIC -O3)
target_include_directories(mppi_trajectory_validators
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(mppi_trajectory_validators PUBLIC
  Eigen3::Eigen
  angles::angles
  geometry_msgs::geometry_msgs
  nav2_core::nav2_core
  nav2_costmap_2d::layers
  nav2_costmap_2d::nav2_costmap_2d_core
  nav_msgs::nav_msgs
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  std_msgs::std_msgs
  tf2::tf2
  tf2_geometry_msgs::tf2_geometry_msgs
  visualization_msgs::visualization_msgs
)
target_link_libraries(mppi_trajectory_validators PRIVATE
  pluginlib::pluginlib
)

install(TARGETS mppi_controller mppi_motion_models mppi_critics mppi_trajectory_validators
  EXPORT nav2_mppi_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_find_gtest()

  add_subdirectory(test)
  add_subdirectory(benchmark)
  if(NAV2_MPPI_CUDA)
    target_compile_definitions(cost_critic_parity_test PRIVATE NAV2_MPPI_HAS_CUDA=1)
  endif()
endif()

ament_export_dependencies(
  angles
  geometry_msgs
  nav2_core
  nav2_costmap_2d
  nav_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  std_msgs
  tf2
  tf2_geometry_msgs
  visualization_msgs
  nav2_ros_common
  Eigen3
)
ament_export_targets(nav2_mppi_controller)

pluginlib_export_plugin_description_file(nav2_core mppic.xml)
pluginlib_export_plugin_description_file(nav2_mppi_controller critics.xml)
pluginlib_export_plugin_description_file(nav2_mppi_controller trajectory_validators.xml)
pluginlib_export_plugin_description_file(nav2_mppi_controller motion_models.xml)

ament_package()
