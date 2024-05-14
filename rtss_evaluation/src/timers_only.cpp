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

#include "rclcpp/rclcpp.hpp"

#include "rtss_evaluation/rt_system.hpp"

#include "rtss_evaluation/timers_only_system_builder.hpp"
#include "rtss_evaluation/default.hpp"
#include "rclcpp/experimental/executors/events_executor/events_executor.hpp"
#include "rclcpp/experimental/executors/graph_executor.hpp"

int main(int argc, char * argv[])
{
  // Check for correct number of arguments
  if (argc < 2) {
    std::cerr << "Usage: " << argv[0] << " <executor type> [RO|RE]" << std::endl;
    return 1;
  }

  rclcpp::init(argc, argv);

  using TimeConfig = rt_nodes::timing::Default;
  // uncomment for benchmarking
  // using TimeConfig = nodes::timing::BenchmarkCPUUsage;
  // set_benchmark_mode(true);

  // Build a set of timers
  auto nodes = create_timer_only_nodes<RTSystem, TimeConfig>();

  // Using argv[3] as a string argument, ascertain which executor we're running
  if (strcmp(argv[1], "rm") == 0) {
    std::cout << "Using rate monotonic fixed priority executor" << std::endl;
    // TODO: Get argv[4] to determine RO or RE

    rclcpp::experimental::executors::GraphExecutor executor = rclcpp::experimental::executors::GraphExecutor();

    for (auto & node : nodes) {
      executor.add_node(node);
    }
    executor.assign_priority();
    executor.spin();
  } else if (strcmp(argv[1], "events") == 0) {
    std::cout << "Using events executor" << std::endl;

    // TODO: Get argv[4] to determine RO or RE

    auto events_queue = std::make_unique<rclcpp::experimental::executors::PriorityEventsQueue>();
    rclcpp::experimental::executors::EventsExecutor executor(std::move(events_queue), false);
    for (auto & node : nodes) {
      executor.add_node(node);
    }
    executor.spin();
  } else {
    std::cout << "Using default executor" << std::endl;

    // Default executor
    rclcpp::executors::SingleThreadedExecutor executor;
    for (auto & node : nodes) {
      executor.add_node(node);
    }
    executor.spin();
  }

  std::cout << "Dropped jobs: " << std::endl;
  for (auto & node : nodes) {
    std::cout << node->get_name() << ": " << node->get_dropped_jobs() << std::endl;
  }

  nodes.clear();
  rclcpp::shutdown();

  return 0;
}
