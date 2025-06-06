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

#include "rtss_evaluation/timers_only_system_builder.hpp"
#include "rtss_evaluation/default.hpp"
#include "rclcpp/experimental/executors/events_executor/events_executor.hpp"
#ifdef RCLCPP_EXPERIMENTAL_GRAPH_EXECUTOR
#include "rclcpp/experimental/executors/graph_executor.hpp"
#endif

int main(int argc, char * argv[])
{
  // Check for correct number of arguments
  if (argc < 3) {
    std::cerr << "Usage: " << argv[0] << "<duration <executor type>" << std::endl;
    return 1;
  }

  rclcpp::init(argc, argv);

  using TimeConfig = rt_nodes::timing::Default;
  // uncomment for benchmarking
  // using TimeConfig = nodes::timing::BenchmarkCPUUsage;
  // set_benchmark_mode(true);

  // Build a set of timers
  auto nodes = create_timers_only_nodes<RTSystem, TimeConfig>();

  int duration = std::stoi(argv[1]);

  // Create a new thread that sleeps for the specified duration
  std::thread shutdown_thread([duration]() {
    struct sched_param params;
    params.sched_priority = 91;
    int ret = pthread_setschedparam(pthread_self(), SCHED_FIFO, &params);
    if (ret != 0) {
      RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Failed to elevate priority of kill thread: %s", strerror(ret));
    }

    std::this_thread::sleep_for(std::chrono::seconds(duration));
    std::cout << "Shutting down" << std::endl;
    rclcpp::shutdown();
  });

  // Using argv[2] as a string argument, ascertain which executor we're running
  if (strcmp(argv[2], "rm") == 0) {
    #if defined(RCLCPP_EXPERIMENTAL_GRAPH_EXECUTOR) && defined(RCLCPP_EXPERIMENTAL_RM_QUEUE)
    auto events_queue = std::make_unique<rclcpp::experimental::executors::RMEventsQueue>();
    rclcpp::experimental::executors::EventsExecutor executor(std::move(events_queue));

    for (auto & node : nodes) {
      executor.add_node(node);
    }
    executor.spin();
    #endif
  } else if (strcmp(argv[2], "edf") == 0) {
    #if defined(RCLCPP_EXPERIMENTAL_GRAPH_EXECUTOR) && defined(RCLCPP_EXPERIMENTAL_EDF_QUEUE)
    auto events_queue = std::make_unique<rclcpp::experimental::executors::EDFEventsQueue>();
    rclcpp::experimental::executors::EventsExecutor executor(std::move(events_queue));
    for (auto & node : nodes) {
      executor.add_node(node);
    }
    executor.spin();
    #endif
  } else if (strcmp(argv[2], "fifo") == 0) {
    auto events_queue = std::make_unique<rclcpp::experimental::executors::SimpleEventsQueue>();
    rclcpp::experimental::executors::EventsExecutor executor(std::move(events_queue));
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
  } else if (strcmp(argv[2], "static") == 0) {
    std::cout << "Using default static executor" << std::endl;

    // Default executor
    rclcpp::executors::StaticSingleThreadedExecutor executor;
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

  uint64_t start_time = UINT64_MAX, end_time = 0;
  for (auto & node : nodes) {
    if (node->get_first_job() < start_time) {
      start_time = node->get_first_job();
    }
    if (node->get_last_job() > end_time) {
      end_time = node->get_last_job();
    }
  }
  uint64_t duration_ns = end_time - start_time;

  uint32_t dropped_jobs_sum = 0;
  uint32_t total_jobs_sum = 0;
  uint32_t total_overrun_jobs = 0;
  std::cout << "Dropped jobs: " << std::endl;
  for (auto & node : nodes) {
    uint64_t completed_jobs = node->get_completed_jobs();
    uint32_t released_jobs = std::max(completed_jobs, duration_ns / node->get_timer_period());
    uint32_t dropped_jobs = released_jobs - completed_jobs;
    uint32_t overrun_jobs = node->get_deadline_overruns();
    std::cout << node->get_name() << ": " << dropped_jobs << " / " << overrun_jobs << " / " << released_jobs << std::endl;
    dropped_jobs_sum += dropped_jobs;
    total_jobs_sum += released_jobs;
    total_overrun_jobs += overrun_jobs;
  }
  std::cout << "Total: " << dropped_jobs_sum << " / " << total_overrun_jobs << " / " << total_jobs_sum << std::endl;

  nodes.clear();
  rclcpp::shutdown();

  return 0;
}
