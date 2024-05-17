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
auto create_timer_only_nodes()
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
        .number_crunch_limit = TimingConfig::LIDAR_DURATION}));

  nodes.emplace_back(
    std::make_shared<typename SystemType::Sensor>(
      rt_nodes::SensorSettings{.node_name = "RearLidarDriver",
        .topic_name = "RearLidarDriver",
        .cycle_time = TimingConfig::LIDAR_DRIVER_PERIOD,
        .number_crunch_limit = TimingConfig::LIDAR_DURATION}));

  nodes.emplace_back(
    std::make_shared<typename SystemType::Sensor>(
      rt_nodes::SensorSettings{.node_name = "FrontCamera",
        .topic_name = "FrontCamera",
        .cycle_time = TimingConfig::CAMERA_DRIVER_PERIOD,
        .number_crunch_limit = TimingConfig::CAMERA_DURATION}));

  nodes.emplace_back(
    std::make_shared<typename SystemType::Sensor>(
      rt_nodes::SensorSettings{.node_name = "RearCamera",
        .topic_name = "RearCamera",
        .cycle_time = TimingConfig::CAMERA_DRIVER_PERIOD,
        .number_crunch_limit = TimingConfig::CAMERA_DURATION}));

  nodes.emplace_back(
    std::make_shared<typename SystemType::Sensor>(
      rt_nodes::SensorSettings{.node_name = "LeftCamera",
        .topic_name = "LeftCamera",
        .cycle_time = TimingConfig::CAMERA_DRIVER_PERIOD,
        .number_crunch_limit = TimingConfig::CAMERA_DURATION}));

  nodes.emplace_back(
    std::make_shared<typename SystemType::Sensor>(
      rt_nodes::SensorSettings{.node_name = "RightCamera",
        .topic_name = "RightCamera",
        .cycle_time = TimingConfig::CAMERA_DRIVER_PERIOD,
        .number_crunch_limit = TimingConfig::CAMERA_DURATION}));

  nodes.emplace_back(
    std::make_shared<typename SystemType::Sensor>(
      rt_nodes::SensorSettings{.node_name = "IMU",
        .topic_name = "imu",
        .cycle_time = TimingConfig::IMU_DRIVER_PERIOD,
        .number_crunch_limit = TimingConfig::IMU_DURATION}));
#pragma GCC diagnostic pop

  return nodes;
}

template<typename SystemType, typename TimingConfig>
auto create_timer_only_nodes_high_utilization()
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
        .number_crunch_limit = TimingConfig::LIDAR_DURATION_OVERUTILIZATED}));

  nodes.emplace_back(
    std::make_shared<typename SystemType::Sensor>(
      rt_nodes::SensorSettings{.node_name = "RearLidarDriver",
        .topic_name = "RearLidarDriver",
        .cycle_time = TimingConfig::LIDAR_DRIVER_PERIOD,
        .number_crunch_limit = TimingConfig::LIDAR_DURATION_OVERUTILIZATED}));

  nodes.emplace_back(
    std::make_shared<typename SystemType::Sensor>(
      rt_nodes::SensorSettings{.node_name = "FrontCamera",
        .topic_name = "FrontCamera",
        .cycle_time = TimingConfig::CAMERA_DRIVER_PERIOD,
        .number_crunch_limit = TimingConfig::CAMERA_DURATION_OVERUTILIZATED}));

  nodes.emplace_back(
    std::make_shared<typename SystemType::Sensor>(
      rt_nodes::SensorSettings{.node_name = "RearCamera",
        .topic_name = "RearCamera",
        .cycle_time = TimingConfig::CAMERA_DRIVER_PERIOD,
        .number_crunch_limit = TimingConfig::CAMERA_DURATION_OVERUTILIZATED}));

  nodes.emplace_back(
    std::make_shared<typename SystemType::Sensor>(
      rt_nodes::SensorSettings{.node_name = "LeftCamera",
        .topic_name = "LeftCamera",
        .cycle_time = TimingConfig::CAMERA_DRIVER_PERIOD,
        .number_crunch_limit = TimingConfig::CAMERA_DURATION_OVERUTILIZATED}));

  nodes.emplace_back(
    std::make_shared<typename SystemType::Sensor>(
      rt_nodes::SensorSettings{.node_name = "RightCamera",
        .topic_name = "RightCamera",
        .cycle_time = TimingConfig::CAMERA_DRIVER_PERIOD,
        .number_crunch_limit = TimingConfig::CAMERA_DURATION_OVERUTILIZATED}));

  nodes.emplace_back(
    std::make_shared<typename SystemType::Sensor>(
      rt_nodes::SensorSettings{.node_name = "IMU",
        .topic_name = "imu",
        .cycle_time = TimingConfig::IMU_DRIVER_PERIOD,
        .number_crunch_limit = TimingConfig::IMU_DURATION}));
#pragma GCC diagnostic pop

  return nodes;
}

template<typename NodeType>
std::shared_ptr<NodeType> get_node(
  const std::string & name,
  const std::vector<std::shared_ptr<NodeType>> & v)
{
  for (const auto & n : v) {
    if (n->get_name() == std::string(name)) {
      return n;
    }
  }

  return std::shared_ptr<NodeType>();
}

#endif  // AUTOWARE_REFERENCE_SYSTEM__TIMERS_ONLY_SYSTEM_BUILDER_HPP_