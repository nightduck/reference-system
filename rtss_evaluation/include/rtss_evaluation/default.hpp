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
#ifndef AUTOWARE_REFERENCE_SYSTEM__SYSTEM__TIMING__DEFAULT_HPP_
#define AUTOWARE_REFERENCE_SYSTEM__SYSTEM__TIMING__DEFAULT_HPP_
#include <chrono>

namespace rt_nodes
{
namespace timing
{

struct Default
{
  using time_t = std::chrono::nanoseconds;
  using milliseconds = std::chrono::milliseconds;

  // sensors
  static constexpr time_t LIDAR_DRIVER_PERIOD = milliseconds(100);
  static constexpr time_t CAMERA_DRIVER_PERIOD = milliseconds(42);
  static constexpr time_t IMU_DRIVER_PERIOD = milliseconds(15);
  static constexpr time_t LIDAR_DRIVER_PERIOD_OVERUTILIZATION = milliseconds(50);
  static constexpr time_t CAMERA_DRIVER_PERIOD_OVERUTILIZATION = milliseconds(30);
  // the following values are used as the number_cruncher_limit
  // to search for primes up to starting at 3
  // for your platform, run the `number_cruncher_benchmark` executable
  // to figure out what values to place here corresponding to the run_time
  // you would like to run each node for
  // processing
  static constexpr uint64_t LIDAR_DURATION = 2048;
  static constexpr uint64_t CAMERA_DURATION = 2048;
  static constexpr uint64_t IMU_DURATION = 512;
  static constexpr uint64_t LIDAR_TRANSFORM_DURATION = 4096;
  static constexpr uint64_t CAMERA_TRANSFORM_DURATION = 4096;
  static constexpr uint64_t LIDAR_DURATION_HIGH_UTILIZATION = 4096;
  static constexpr uint64_t CAMERA_DURATION_HIGH_UTILIZATION = 3072;
  static constexpr uint64_t LIDAR_WCET = 4900000;
  static constexpr uint64_t CAMERA_WCET = 4900000;
  static constexpr uint64_t IMU_WCET = 400000;
  static constexpr uint64_t LIDAR_WCET_HIGH_UTILIZATION = 9900000;
  static constexpr uint64_t CAMERA_WCET_HIGH_UTILIZATION = 6900000;
};

constexpr Default::time_t Default::LIDAR_DRIVER_PERIOD;
constexpr Default::time_t Default::CAMERA_DRIVER_PERIOD;
constexpr Default::time_t Default::IMU_DRIVER_PERIOD;
constexpr Default::time_t Default::LIDAR_DRIVER_PERIOD_OVERUTILIZATION;
constexpr Default::time_t Default::CAMERA_DRIVER_PERIOD_OVERUTILIZATION;
constexpr uint64_t Default::LIDAR_DURATION;
constexpr uint64_t Default::CAMERA_DURATION;
constexpr uint64_t Default::IMU_DURATION;
constexpr uint64_t Default::LIDAR_TRANSFORM_DURATION;
constexpr uint64_t Default::CAMERA_TRANSFORM_DURATION;
constexpr uint64_t Default::LIDAR_WCET;
constexpr uint64_t Default::CAMERA_WCET;
constexpr uint64_t Default::IMU_WCET;
constexpr uint64_t Default::LIDAR_DURATION_HIGH_UTILIZATION;
constexpr uint64_t Default::CAMERA_DURATION_HIGH_UTILIZATION;

}  // namespace timing
}  // namespace rt_nodes
#endif  // AUTOWARE_REFERENCE_SYSTEM__SYSTEM__TIMING__DEFAULT_HPP_
