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

#include <thread>

#include "rclcpp/rclcpp.hpp"

#include "rtss_evaluation/rt_system.hpp"

#include "rtss_evaluation/timers_only_system_builder.hpp"
#include "rtss_evaluation/default.hpp"
#include "rclcpp/experimental/executors/events_executor/events_executor.hpp"
#include "rclcpp/experimental/executors/graph_executor.hpp"

int main(int argc, char * argv[])
{
  // Check for correct number of arguments
  if (argc < 3) {
    std::cerr << "Usage: " << argv[0] << "<duration <executor type> [RO|RE]" << std::endl;
    return 1;
  }

  rclcpp::init(argc, argv);

  using TimeConfig = rt_nodes::timing::Default;
  // uncomment for benchmarking
  // using TimeConfig = nodes::timing::BenchmarkCPUUsage;
  // set_benchmark_mode(true);

  // Build a set of timers
  auto nodes = create_timer_only_nodes<RTSystem, TimeConfig>();

  int duration = std::stoi(argv[1]);

  // Create a new thread that sleeps for the specified duration
  std::thread shutdown_thread([duration]() {
    std::this_thread::sleep_for(std::chrono::seconds(duration));
    rclcpp::shutdown();
  });

  // Using argv[2] as a string argument, ascertain which executor we're running
  if (strcmp(argv[2], "rm") == 0) {

    // Get argv[4] to determine RO or RE
    bool execute_timers_separate_thread = false;
    if (argc >= 3 && strcmp(argv[3], "RE") == 0) {
      execute_timers_separate_thread = true;
      std::cout << "Using rate monotonic fixed priority executor in RE mode" << std::endl;
    } else {
      std::cout << "Using rate monotonic fixed priority executor in RO mode" << std::endl;
    }

    rclcpp::experimental::executors::GraphExecutor executor
      = rclcpp::experimental::executors::GraphExecutor(execute_timers_separate_thread);

    for (auto & node : nodes) {
      executor.add_node(node);
    }
    executor.assign_priority();
    executor.spin();
  } else if (strcmp(argv[2], "edf") == 0) {
    // Get argv[4] to determine RO or RE
    bool execute_timers_separate_thread = false;
    if (argc >= 3 && strcmp(argv[3], "RE") == 0) {
      execute_timers_separate_thread = true;
      std::cout << "Using events executor in RE mode" << std::endl;
    } else {
      std::cout << "Using events executor in RO mode" << std::endl;
    }

    auto events_queue = std::make_unique<rclcpp::experimental::executors::EDFEventsQueue>();
    rclcpp::experimental::executors::EventsExecutor executor(std::move(events_queue), execute_timers_separate_thread);
    for (auto & node : nodes) {
      executor.add_node(node);
    }
    executor.spin();
  } else if (strcmp(argv[2], "events") == 0) {
    // Get argv[4] to determine RO or RE
    bool execute_timers_separate_thread = false;
    if (argc >= 3 && strcmp(argv[3], "RE") == 0) {
      execute_timers_separate_thread = true;
      std::cout << "Using events executor in RE mode" << std::endl;
    } else {
      std::cout << "Using events executor in RO mode" << std::endl;
    }

    auto events_queue = std::make_unique<rclcpp::experimental::executors::PriorityEventsQueue>();
    rclcpp::experimental::executors::EventsExecutor executor(std::move(events_queue), execute_timers_separate_thread);
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

  // Wait for the thread to finish
  shutdown_thread.join();

  uint32_t dropped_jobs_sum = 0;
  uint32_t total_jobs_sum = 0;
  std::cout << "Dropped jobs: " << std::endl;
  for (auto & node : nodes) {
    uint32_t dropped_jobs = node->get_dropped_jobs();
    uint32_t total_jobs = node->get_completed_jobs() + node->get_dropped_jobs();
    std::cout << node->get_name() << ": " << dropped_jobs << " / " << total_jobs << std::endl;
    dropped_jobs_sum += dropped_jobs;
    total_jobs_sum += total_jobs;
  }
  std::cout << "Total: " << dropped_jobs_sum << " / " << total_jobs_sum << std::endl;

  nodes.clear();
  rclcpp::shutdown();

  return 0;
}