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
#include "rclcpp/experimental/experimental_definitions.hpp"

#include "rtss_evaluation/rt_system.hpp"

#include "rtss_evaluation/graph_system_builder.hpp"
#include "rtss_evaluation/default.hpp"
#include "rclcpp/experimental/executors/events_executor/events_executor.hpp"
#ifdef RCLCPP_EXPERIMENTAL_GRAPH_EXECUTOR
#include "rclcpp/experimental/executors/graph_executor.hpp"
#endif
#ifdef RCLCPP_EXPERIMENTAL_PERIOD_QUEUE
#include "rclcpp/experimental/executors/events_executor/period_events_queue.hpp"
#endif
#ifdef RCLCPP_EXPERIMENTAL_DEADLINE_QUEUE
#include "rclcpp/experimental/executors/events_executor/deadline_events_queue.hpp"
#endif

int main(int argc, char * argv[])
{
  // Check for correct number of arguments
  if (argc < 3) {
    std::cerr << "Usage: " << argv[0] << "<duration> <executor type> [uu/ou]" << std::endl;
    return 1;
  }

  rclcpp::init(argc, argv);

  using TimeConfig = rt_nodes::timing::Default;
  // uncomment for benchmarking
  // using TimeConfig = nodes::timing::BenchmarkCPUUsage;
  // set_benchmark_mode(true);

  // Build a set of nodes
  std::vector<std::shared_ptr<RTSystem::NodeBaseType>> nodes;
  if (argc == 4 && strcmp(argv[3], "uu") == 0) {
    std::cout << "Using underutilized nodes" << std::endl;
    nodes = create_graph_nodes_underutilized<RTSystem, TimeConfig>();
  } else if (argc == 4 && strcmp(argv[3], "ou") == 0) {
    std::cout << "Using overutilized nodes" << std::endl;
    nodes = create_graph_nodes_overutilized<RTSystem, TimeConfig>();
  } else if (argc == 4 && strcmp(argv[3], "human") == 0) {
    std::cout << "Using real-time nodes" << std::endl;
    nodes = create_graph_nodes_human_speed<RTSystem, TimeConfig>();
  } else {
    std::cout << "Using normal nodes" << std::endl;
    nodes = create_graph_nodes<RTSystem, TimeConfig>();
  }
  int duration = std::stoi(argv[1]);

  // Create a new thread that sleeps for the specified duration
  std::thread shutdown_thread([duration]() {
    std::this_thread::sleep_for(std::chrono::seconds(duration));
    std::cout << "Shutting down" << std::endl;
    rclcpp::shutdown();
  });

  // Using argv[2] as a string argument, ascertain which executor we're running
  if (strcmp(argv[2], "rm") == 0) {
    #if defined(RCLCPP_EXPERIMENTAL_GRAPH_EXECUTOR) && defined(RCLCPP_EXPERIMENTAL_RM_QUEUE)
    // Get argv[4] to determine RO or RE
    rclcpp::experimental::executors::RMEventsQueue::UniquePtr event_queue =
      std::make_unique<rclcpp::experimental::executors::RMEventsQueue>();
    rclcpp::experimental::executors::GraphExecutor executor(std::move(event_queue));
    #elif defined(RCLCPP_EXPERIMENTAL_PERIOD_QUEUE)
    rclcpp::experimental::executors::PeriodEventsQueue::UniquePtr event_queue =
      std::make_unique<rclcpp::experimental::executors::PeriodEventsQueue>();
    rclcpp::experimental::executors::EventsExecutor executor(std::move(event_queue));
    #endif

    for (auto & node : nodes) {
      executor.add_node(node);
    }
    executor.spin();
  } else if (strcmp(argv[2], "edf") == 0) {
    #if defined(RCLCPP_EXPERIMENTAL_GRAPH_EXECUTOR) && defined(RCLCPP_EXPERIMENTAL_EDF_QUEUE)
    // Get argv[4] to determine RO or RE
    rclcpp::experimental::executors::EDFEventsQueue::UniquePtr event_queue =
      std::make_unique<rclcpp::experimental::executors::EDFEventsQueue>();
    rclcpp::experimental::executors::GraphExecutor executor(std::move(event_queue));
    #elif defined(RCLCPP_EXPERIMENTAL_DEADLINE_QUEUE)
    auto events_queue = std::make_unique<rclcpp::experimental::executors::DeadlineEventsQueue>();
    rclcpp::experimental::executors::EventsExecutor executor(std::move(events_queue));
    #endif
    for (auto & node : nodes) {
      executor.add_node(node);
    }
    executor.spin();
  } else if (strcmp(argv[2], "events") == 0) {
    auto events_queue = std::make_unique<rclcpp::experimental::executors::SimpleEventsQueue>();

    rclcpp::experimental::executors::EventsExecutor executor(std::move(events_queue));
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

  std::cout << "Executor finished" << std::endl;

  // Wait for the thread to finish
  shutdown_thread.join();

  std::cout << "Dropped jobs:" << std::endl;

  int total_timer_jobs = 0;
  uint32_t dropped_jobs_sum = 0;
  uint32_t total_jobs_sum = 0;
  for (auto & node : nodes) {
    if (node->get_timer_period() > 0) {
      total_timer_jobs += node->get_completed_jobs() + node->get_dropped_jobs();
      std::cout << node->get_name() << ": " << node->get_dropped_jobs() << " / "
        << node->get_completed_jobs() + node->get_dropped_jobs() << std::endl;

      dropped_jobs_sum += node->get_dropped_jobs();
      total_jobs_sum += node->get_completed_jobs();
    }
  }

  for (auto & node : nodes) {
    if (node->get_timer_period() == 0) {
      std::cout << node->get_name() << ": " << total_timer_jobs - node->get_completed_jobs() << " / " 
        << total_timer_jobs << std::endl;

      dropped_jobs_sum += total_timer_jobs - node->get_completed_jobs();
      total_jobs_sum += total_timer_jobs;
    }
  }

  std::cout << "Total: " << dropped_jobs_sum << " / " << total_jobs_sum << std::endl;

  nodes.clear();
  rclcpp::shutdown();

  return 0;
}
