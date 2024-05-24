add_project_private_dependency(benchmark REQUIRED)

add_custom_target(bench)

function(ADD_SIMPLE_BENCHMARK benchmark_name)
  add_executable(${benchmark_name} ${benchmark_name}.cpp)
  target_link_libraries(${benchmark_name} PRIVATE ${PROJECT_NAME})
  target_link_libraries(${benchmark_name} PRIVATE benchmark::benchmark)

  add_dependencies(bench ${benchmark_name})
endfunction()

add_simple_benchmark(mujoco-humanoid)
add_simple_benchmark(affine-transform)
