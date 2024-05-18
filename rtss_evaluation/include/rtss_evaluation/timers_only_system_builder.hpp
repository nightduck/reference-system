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
#ifndef AUTOWARE_REFERENCE_SYSTEM__TIMERS_ONLY_SYSTEM_BUILDER_HPP_
#define AUTOWARE_REFERENCE_SYSTEM__TIMERS_ONLY_SYSTEM_BUILDER_HPP_
#include <chrono>
#include <memory>
#include <vector>
#include <string>

#include "settings.hpp"
#include "reference_system/sample_management.hpp"

template<typename SystemType, typename TimingConfig>
auto create_timers_only_nodes()
->std::vector<std::shared_ptr<typename SystemType::Sensor>>
{
  std::vector<std::shared_ptr<typename SystemType::Sensor>> nodes;

// ignore the warning about designated initializers - they make the code much
// more readable
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"

  // setup communication graph
  // sensor nodes
  nodes.emplace_back(
    std::make_shared<typename SystemType::Sensor>(
      rt_nodes::SensorSettings{.node_name = "FrontLidarDriver",
        .topic_name = "FrontLidarDriver",
        .cycle_time = TimingConfig::LIDAR_DRIVER_PERIOD,
        .number_crunch_limit = TimingConfig::DEFAULT_DURATION,
        .wcet = TimingConfig::LIDAR_WCET}));

  nodes.emplace_back(
    std::make_shared<typename SystemType::Sensor>(
      rt_nodes::SensorSettings{.node_name = "RearLidarDriver",
        .topic_name = "RearLidarDriver",
        .cycle_time = TimingConfig::LIDAR_DRIVER_PERIOD,
        .number_crunch_limit = TimingConfig::DEFAULT_DURATION,
        .wcet = TimingConfig::LIDAR_WCET}));

  nodes.emplace_back(
    std::make_shared<typename SystemType::Sensor>(
      rt_nodes::SensorSettings{.node_name = "FrontCamera",
        .topic_name = "FrontCamera",
        .cycle_time = TimingConfig::CAMERA_DRIVER_PERIOD,
        .number_crunch_limit = TimingConfig::DEFAULT_DURATION,
        .wcet = TimingConfig::CAMERA_WCET}));

  nodes.emplace_back(
    std::make_shared<typename SystemType::Sensor>(
      rt_nodes::SensorSettings{.node_name = "RearCamera",
        .topic_name = "RearCamera",
        .cycle_time = TimingConfig::CAMERA_DRIVER_PERIOD,
        .number_crunch_limit = TimingConfig::DEFAULT_DURATION,
        .wcet = TimingConfig::CAMERA_WCET}));

  nodes.emplace_back(
    std::make_shared<typename SystemType::Sensor>(
      rt_nodes::SensorSettings{.node_name = "LeftCamera",
        .topic_name = "LeftCamera",
        .cycle_time = TimingConfig::CAMERA_DRIVER_PERIOD,
        .number_crunch_limit = TimingConfig::DEFAULT_DURATION,
        .wcet = TimingConfig::CAMERA_WCET}));

  nodes.emplace_back(
    std::make_shared<typename SystemType::Sensor>(
      rt_nodes::SensorSettings{.node_name = "RightCamera",
        .topic_name = "RightCamera",
        .cycle_time = TimingConfig::CAMERA_DRIVER_PERIOD,
        .number_crunch_limit = TimingConfig::DEFAULT_DURATION,
        .wcet = TimingConfig::CAMERA_WCET}));

  nodes.emplace_back(
    std::make_shared<typename SystemType::Sensor>(
      rt_nodes::SensorSettings{.node_name = "IMU",
        .topic_name = "imu",
        .cycle_time = TimingConfig::IMU_DRIVER_PERIOD,
        .number_crunch_limit = TimingConfig::IMU_DURATION,
        .wcet = TimingConfig::IMU_WCET}));
#pragma GCC diagnostic pop

  return nodes;
}

template<typename SystemType, typename TimingConfig>
auto create_timers_only_nodes_high_utilization()
->std::vector<std::shared_ptr<typename SystemType::Sensor>>
{
  std::vector<std::shared_ptr<typename SystemType::Sensor>> nodes;

// ignore the warning about designated initializers - they make the code much
// more readable
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"

  // setup communication graph
  // sensor nodes
  nodes.emplace_back(
    std::make_shared<typename SystemType::Sensor>(
      rt_nodes::SensorSettings{.node_name = "FrontLidarDriver",
        .topic_name = "FrontLidarDriver",
        .cycle_time = TimingConfig::LIDAR_DRIVER_PERIOD,
        .number_crunch_limit = TimingConfig::DEFAULT_DURATION,
        .wcet = TimingConfig::LIDAR_WCET_HIGH_UTILIZATION}));

  nodes.emplace_back(
    std::make_shared<typename SystemType::Sensor>(
      rt_nodes::SensorSettings{.node_name = "RearLidarDriver",
        .topic_name = "RearLidarDriver",
        .cycle_time = TimingConfig::LIDAR_DRIVER_PERIOD,
        .number_crunch_limit = TimingConfig::DEFAULT_DURATION,
        .wcet = TimingConfig::LIDAR_WCET_HIGH_UTILIZATION}));

  nodes.emplace_back(
    std::make_shared<typename SystemType::Sensor>(
      rt_nodes::SensorSettings{.node_name = "FrontCamera",
        .topic_name = "FrontCamera",
        .cycle_time = TimingConfig::CAMERA_DRIVER_PERIOD_HIGH_UTILIZATION,
        .number_crunch_limit = TimingConfig::DEFAULT_DURATION,
        .wcet = TimingConfig::CAMERA_WCET_HIGH_UTILIZATION}));

  nodes.emplace_back(
    std::make_shared<typename SystemType::Sensor>(
      rt_nodes::SensorSettings{.node_name = "RearCamera",
        .topic_name = "RearCamera",
        .cycle_time = TimingConfig::CAMERA_DRIVER_PERIOD,
        .number_crunch_limit = TimingConfig::DEFAULT_DURATION,
        .wcet = TimingConfig::CAMERA_WCET_HIGH_UTILIZATION}));

  nodes.emplace_back(
    std::make_shared<typename SystemType::Sensor>(
      rt_nodes::SensorSettings{.node_name = "LeftCamera",
        .topic_name = "LeftCamera",
        .cycle_time = TimingConfig::CAMERA_DRIVER_PERIOD,
        .number_crunch_limit = TimingConfig::DEFAULT_DURATION,
        .wcet = TimingConfig::CAMERA_WCET_HIGH_UTILIZATION}));

  nodes.emplace_back(
    std::make_shared<typename SystemType::Sensor>(
      rt_nodes::SensorSettings{.node_name = "RightCamera",
        .topic_name = "RightCamera",
        .cycle_time = TimingConfig::CAMERA_DRIVER_PERIOD,
        .number_crunch_limit = TimingConfig::DEFAULT_DURATION,
        .wcet = TimingConfig::CAMERA_WCET_HIGH_UTILIZATION}));

  nodes.emplace_back(
    std::make_shared<typename SystemType::Sensor>(
      rt_nodes::SensorSettings{.node_name = "IMU",
        .topic_name = "imu",
        .cycle_time = TimingConfig::IMU_DRIVER_PERIOD,
        .number_crunch_limit = TimingConfig::IMU_DURATION,
        .wcet = TimingConfig::IMU_WCET}));
#pragma GCC diagnostic pop

  return nodes;
}

template<typename SystemType, typename TimingConfig>
auto create_timers_only_nodes_over_utilization()
->std::vector<std::shared_ptr<typename SystemType::Sensor>>
{
  std::vector<std::shared_ptr<typename SystemType::Sensor>> nodes;

// ignore the warning about designated initializers - they make the code much
// more readable
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"

  // setup communication graph
  // sensor nodes
  nodes.emplace_back(
    std::make_shared<typename SystemType::Sensor>(
      rt_nodes::SensorSettings{.node_name = "FrontLidarDriver",
        .topic_name = "FrontLidarDriver",
        .cycle_time = TimingConfig::LIDAR_DRIVER_PERIOD_OVERUTILIZATION,
        .number_crunch_limit = TimingConfig::DEFAULT_DURATION,
        .wcet = TimingConfig::LIDAR_WCET_OVER_UTILIZATION}));

  nodes.emplace_back(
    std::make_shared<typename SystemType::Sensor>(
      rt_nodes::SensorSettings{.node_name = "RearLidarDriver",
        .topic_name = "RearLidarDriver",
        .cycle_time = TimingConfig::LIDAR_DRIVER_PERIOD_OVERUTILIZATION,
        .number_crunch_limit = TimingConfig::DEFAULT_DURATION,
        .wcet = TimingConfig::LIDAR_WCET_OVER_UTILIZATION}));

  nodes.emplace_back(
    std::make_shared<typename SystemType::Sensor>(
      rt_nodes::SensorSettings{.node_name = "FrontCamera",
        .topic_name = "FrontCamera",
        .cycle_time = TimingConfig::CAMERA_DRIVER_PERIOD_OVERUTILIZATION,
        .number_crunch_limit = TimingConfig::DEFAULT_DURATION,
        .wcet = TimingConfig::CAMERA_WCET_OVER_UTILIZATION}));

  nodes.emplace_back(
    std::make_shared<typename SystemType::Sensor>(
      rt_nodes::SensorSettings{.node_name = "RearCamera",
        .topic_name = "RearCamera",
        .cycle_time = TimingConfig::CAMERA_DRIVER_PERIOD_OVERUTILIZATION,
        .number_crunch_limit = TimingConfig::DEFAULT_DURATION,
        .wcet = TimingConfig::CAMERA_WCET_OVER_UTILIZATION}));

  nodes.emplace_back(
    std::make_shared<typename SystemType::Sensor>(
      rt_nodes::SensorSettings{.node_name = "LeftCamera",
        .topic_name = "LeftCamera",
        .cycle_time = TimingConfig::CAMERA_DRIVER_PERIOD_OVERUTILIZATION,
        .number_crunch_limit = TimingConfig::DEFAULT_DURATION,
        .wcet = TimingConfig::CAMERA_WCET_OVER_UTILIZATION}));

  nodes.emplace_back(
    std::make_shared<typename SystemType::Sensor>(
      rt_nodes::SensorSettings{.node_name = "RightCamera",
        .topic_name = "RightCamera",
        .cycle_time = TimingConfig::CAMERA_DRIVER_PERIOD_OVERUTILIZATION,
        .number_crunch_limit = TimingConfig::DEFAULT_DURATION,
        .wcet = TimingConfig::CAMERA_WCET_OVER_UTILIZATION}));

  nodes.emplace_back(
    std::make_shared<typename SystemType::Sensor>(
      rt_nodes::SensorSettings{.node_name = "UnderCamera",
        .topic_name = "UnderCamera",
        .cycle_time = TimingConfig::CAMERA_DRIVER_PERIOD_OVERUTILIZATION,
        .number_crunch_limit = TimingConfig::DEFAULT_DURATION,
        .wcet = TimingConfig::CAMERA_WCET_HIGH_UTILIZATION}));

  // nodes.emplace_back(
  //   std::make_shared<typename SystemType::Sensor>(
  //     rt_nodes::SensorSettings{.node_name = "OverCamera",
  //       .topic_name = "OverCamera",
  //       .cycle_time = TimingConfig::CAMERA_DRIVER_PERIOD_OVERUTILIZATION,
  //       .number_crunch_limit = TimingConfig::DEFAULT_DURATION,
  //       .wcet = TimingConfig::CAMERA_WCET_HIGH_UTILIZATION}));

  nodes.emplace_back(
    std::make_shared<typename SystemType::Sensor>(
      rt_nodes::SensorSettings{.node_name = "IMU",
        .topic_name = "imu",
        .cycle_time = TimingConfig::IMU_DRIVER_PERIOD,
        .number_crunch_limit = TimingConfig::DEFAULT_DURATION,
        .wcet = TimingConfig::IMU_WCET_OVER_UTILIZATION}));
#pragma GCC diagnostic pop

  return nodes;
}

#endif  // AUTOWARE_REFERENCE_SYSTEM__TIMERS_ONLY_SYSTEM_BUILDER_HPP_