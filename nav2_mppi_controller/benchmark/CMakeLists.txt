find_package(benchmark REQUIRED)

set(BENCHMARK_NAMES
  optimizer_benchmark
  controller_benchmark
  mppi_gpu_baseline
)

foreach(name IN LISTS BENCHMARK_NAMES)
  add_executable(${name}
    ${name}.cpp
  )
  target_link_libraries(${name}
    benchmark
    geometry_msgs::geometry_msgs
    mppi_controller
    mppi_critics
    nav2_core::nav2_core
    nav2_costmap_2d::nav2_costmap_2d_core
    nav_msgs::nav_msgs
  )

  target_include_directories(${name} PRIVATE
    ${PROJECT_SOURCE_DIR}/test/utils
  )
endforeach()

if(NAV2_MPPI_CUDA)
  add_executable(cost_critic_cuda_benchmark
    cost_critic_cuda_benchmark.cpp
  )
  target_link_libraries(cost_critic_cuda_benchmark
    benchmark
    geometry_msgs::geometry_msgs
    mppi_controller
    mppi_critics
    nav2_core::nav2_core
    nav2_costmap_2d::nav2_costmap_2d_core
    nav_msgs::nav_msgs
    CUDA::cudart
  )
  target_compile_definitions(cost_critic_cuda_benchmark PRIVATE NAV2_MPPI_HAS_CUDA=1)
endif()

if(NAV2_MPPI_CUDA)
  add_executable(obstacles_critic_cuda_benchmark
    obstacles_critic_cuda_benchmark.cpp
  )
  target_link_libraries(obstacles_critic_cuda_benchmark
    benchmark
    geometry_msgs::geometry_msgs
    mppi_controller
    mppi_critics
    nav2_core::nav2_core
    nav2_costmap_2d::nav2_costmap_2d_core
    nav_msgs::nav_msgs
    CUDA::cudart
  )
  target_compile_definitions(obstacles_critic_cuda_benchmark PRIVATE NAV2_MPPI_HAS_CUDA=1)
endif()
