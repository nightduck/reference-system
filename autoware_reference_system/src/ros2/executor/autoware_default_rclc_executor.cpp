// Copyright 2021 Apex.AI, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "stdio.h"
#include "rclcpp/rclcpp.hpp"

#include "reference_system/system/systems.hpp"

#include "autoware_reference_system/autoware_system_builder.hpp"
#include "autoware_reference_system/system/timing/benchmark.hpp"
#include "autoware_reference_system/system/timing/default.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  using TimeConfig = nodes::timing::Default;
  // uncomment for benchmarking
  // using TimeConfig = nodes::timing::BenchmarkCPUUsage;
  // set_benchmark_mode(true);

  auto nodes = create_autoware_nodes<RclcSystem, TimeConfig>();

  rclcpp::executors::SingleThreadedExecutor executor;

  rclc_executor_t rclcExecutor = rclc_executor_get_zero_initialized_executor();
  unsigned int num_handles = 2;
  rcl_context_t context = rcl_get_zero_initialized_context();
  rcl_allocator_t allocator = rcl_get_default_allocator();
  rclc_executor_init(&rclcExecutor, &context, num_handles, &allocator);

  for (auto & node : nodes) {
    if (node.get_name() == "PointCloudFusion") {
      printf("... added PointCloudFusion to rclc-executor.\n");
      node.add_to_executor(&rclcExecutor);
    } else {
      executor.add_node(node);
    }
  }

  // TODO check ros context in while condition
  uint64_t timeout_ns = 1000000000; // 1s
  while (1) {
    executor.spin_some();
    rclc_executor_spin_some(&rclcExecutor,timeout_ns);
  }

  nodes.clear();
  rclcpp::shutdown();

  return 0;
}
