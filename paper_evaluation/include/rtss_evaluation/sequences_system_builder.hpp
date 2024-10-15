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
#ifndef AUTOWARE_REFERENCE_SYSTEM__SEQUENCES_SYSTEM_BUILDER_HPP_
#define AUTOWARE_REFERENCE_SYSTEM__SEQUENCES_SYSTEM_BUILDER_HPP_
#include <chrono>
#include <memory>
#include <vector>
#include <string>

#include "settings.hpp"
#include "reference_system/sample_management.hpp"

template<typename SystemType, typename TimingConfig>
auto create_sequences_nodes()
->std::vector<std::shared_ptr<typename SystemType::NodeBaseType>>
{
  std::vector<std::shared_ptr<typename SystemType::NodeBaseType>> nodes;

// ignore the warning about designated initializers - they make the code much
// more readable
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"

  // setup communication graph
  // sensor nodes
  nodes.emplace_back(
    std::make_shared<typename SystemType::Sensor>(
      rt_nodes::SensorSettings{.node_name = "LidarDriver",
        .topic_name = "LidarDriver",
        .cycle_time = TimingConfig::LIDAR_DRIVER_PERIOD,
        .number_crunch_limit = TimingConfig::LIDAR_DURATION,
        .wcet = TimingConfig::LIDAR_WCET}));

  nodes.emplace_back(
    std::make_shared<typename SystemType::Transform>(
      rt_nodes::TransformSettings{.node_name = "PointsTransformer",
        .input_topic = "LidarDriver",
        .output_topic = "LidarTransform",
        .number_crunch_limit = TimingConfig::LIDAR_TRANSFORM_DURATION,
        .cycle_time = TimingConfig::LIDAR_DRIVER_PERIOD,
        .wcet = TimingConfig::LIDAR_TRANSFORM_WCET}));

  nodes.emplace_back(
    std::make_shared<typename SystemType::Transform>(
      rt_nodes::TransformSettings{.node_name = "Localization",
        .input_topic = "LidarTransform",
        .output_topic = "Position",
        .number_crunch_limit = TimingConfig::LOCALIZATION_DURATION,
        .cycle_time = TimingConfig::LIDAR_DRIVER_PERIOD,
        .wcet = TimingConfig::LOCALIZATION_WCET}));

  nodes.emplace_back(
    std::make_shared<typename SystemType::Sensor>(
      rt_nodes::SensorSettings{.node_name = "FrontCamera",
        .topic_name = "FrontCameraRaw",
        .cycle_time = TimingConfig::CAMERA_DRIVER_PERIOD,
        .number_crunch_limit = TimingConfig::CAMERA_DURATION,
        .wcet = TimingConfig::CAMERA_WCET}));

  nodes.emplace_back(
    std::make_shared<typename SystemType::Transform>(
      rt_nodes::TransformSettings{.node_name = "ImageProcessingFront",
        .input_topic = "FrontCameraRaw",
        .output_topic = "FrontCamera",
        .number_crunch_limit = TimingConfig::CAMERA_TRANSFORM_DURATION,
        .cycle_time = TimingConfig::CAMERA_DRIVER_PERIOD,
        .wcet = TimingConfig::CAMERA_TRANSFORM_WCET}));

  nodes.emplace_back(
    std::make_shared<typename SystemType::Transform>(
      rt_nodes::TransformSettings{.node_name = "LaneDetection",
        .input_topic = "FrontCamera",
        .output_topic = "Setpoint",
        .number_crunch_limit = TimingConfig::CAMERA_TRANSFORM_DURATION,
        .cycle_time = TimingConfig::CAMERA_DRIVER_PERIOD,
        .wcet = TimingConfig::CAMERA_TRANSFORM_WCET}));

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
auto create_sequences_nodes_high_utilization()
->std::vector<std::shared_ptr<typename SystemType::NodeBaseType>>
{
  std::vector<std::shared_ptr<typename SystemType::NodeBaseType>> nodes;

// ignore the warning about designated initializers - they make the code much
// more readable
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"

  // setup communication graph
  // sensor nodes
  nodes.emplace_back(
    std::make_shared<typename SystemType::Sensor>(
      rt_nodes::SensorSettings{.node_name = "LidarDriverFront",
        .topic_name = "LidarFront",
        .cycle_time = TimingConfig::LIDAR_DRIVER_PERIOD,
        .number_crunch_limit = TimingConfig::DEFAULT_DURATION,
        .wcet = TimingConfig::LIDAR_WCET}));

  nodes.emplace_back(
    std::make_shared<typename SystemType::Transform>(
      rt_nodes::TransformSettings{.node_name = "PointsTransformerFront",
        .input_topic = "LidarFront",
        .output_topic = "LidarTransformFront",
        .number_crunch_limit = TimingConfig::DEFAULT_DURATION,
        .cycle_time = TimingConfig::LIDAR_DRIVER_PERIOD,
        .wcet = TimingConfig::LIDAR_TRANSFORM_WCET}));

  nodes.emplace_back(
    std::make_shared<typename SystemType::Transform>(
      rt_nodes::TransformSettings{.node_name = "Localization",
        .input_topic = "LidarTransformFront",
        .output_topic = "Position",
        .number_crunch_limit = TimingConfig::LOCALIZATION_DURATION,
        .cycle_time = TimingConfig::LIDAR_DRIVER_PERIOD,
        .wcet = TimingConfig::LOCALIZATION_WCET}));

  nodes.emplace_back(
    std::make_shared<typename SystemType::Sensor>(
      rt_nodes::SensorSettings{.node_name = "LidarDriverRear",
        .topic_name = "LidarRear",
        .cycle_time = TimingConfig::LIDAR_DRIVER_PERIOD,
        .number_crunch_limit = TimingConfig::DEFAULT_DURATION,
        .wcet = TimingConfig::LIDAR_WCET}));

  nodes.emplace_back(
    std::make_shared<typename SystemType::Transform>(
      rt_nodes::TransformSettings{.node_name = "PointsTransformerRear",
        .input_topic = "LidarRear",
        .output_topic = "LidarTransformRear",
        .number_crunch_limit = TimingConfig::DEFAULT_DURATION,
        .cycle_time = TimingConfig::LIDAR_DRIVER_PERIOD,
        .wcet = TimingConfig::LIDAR_TRANSFORM_WCET}));

  nodes.emplace_back(
    std::make_shared<typename SystemType::Sensor>(
      rt_nodes::SensorSettings{.node_name = "FrontCamera",
        .topic_name = "FrontCameraRaw",
        .cycle_time = TimingConfig::CAMERA_DRIVER_PERIOD,
        .number_crunch_limit = TimingConfig::CAMERA_DURATION,
        .wcet = TimingConfig::CAMERA_WCET}));

  nodes.emplace_back(
    std::make_shared<typename SystemType::Transform>(
      rt_nodes::TransformSettings{.node_name = "ImageProcessingFront",
        .input_topic = "FrontCameraRaw",
        .output_topic = "FrontCamera",
        .number_crunch_limit = TimingConfig::CAMERA_TRANSFORM_DURATION,
        .cycle_time = TimingConfig::CAMERA_DRIVER_PERIOD,
        .wcet = TimingConfig::CAMERA_TRANSFORM_WCET}));

  nodes.emplace_back(
    std::make_shared<typename SystemType::Transform>(
      rt_nodes::TransformSettings{.node_name = "LaneDetection",
        .input_topic = "FrontCamera",
        .output_topic = "Setpoint",
        .number_crunch_limit = TimingConfig::CAMERA_TRANSFORM_DURATION,
        .cycle_time = TimingConfig::CAMERA_DRIVER_PERIOD,
        .wcet = TimingConfig::CAMERA_TRANSFORM_WCET}));

  nodes.emplace_back(
    std::make_shared<typename SystemType::Sensor>(
      rt_nodes::SensorSettings{.node_name = "RearCamera",
        .topic_name = "RearCameraRaw",
        .cycle_time = TimingConfig::CAMERA_DRIVER_PERIOD,
        .number_crunch_limit = TimingConfig::CAMERA_DURATION,
        .wcet = TimingConfig::CAMERA_WCET}));

  nodes.emplace_back(
    std::make_shared<typename SystemType::Transform>(
      rt_nodes::TransformSettings{.node_name = "ImageProcessingRear",
        .input_topic = "RearCameraRaw",
        .output_topic = "RearCamera",
        .number_crunch_limit = TimingConfig::CAMERA_TRANSFORM_DURATION,
        .cycle_time = TimingConfig::CAMERA_DRIVER_PERIOD,
        .wcet = TimingConfig::CAMERA_TRANSFORM_WCET}));

  nodes.emplace_back(
    std::make_shared<typename SystemType::Sensor>(
      rt_nodes::SensorSettings{.node_name = "IMU",
        .topic_name = "imu",
        .cycle_time = TimingConfig::IMU_DRIVER_PERIOD,
        .number_crunch_limit = TimingConfig::IMU_DURATION,
        .wcet = TimingConfig::IMU_WCET_OVER_UTILIZATION}));
#pragma GCC diagnostic pop

  return nodes;
}

#endif  // AUTOWARE_REFERENCE_SYSTEM__SEQUENCES_SYSTEM_BUILDER_HPP_