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
#ifndef AUTOWARE_REFERENCE_SYSTEM__GRAPH_SYSTEM_BUILDER_HPP_
#define AUTOWARE_REFERENCE_SYSTEM__GRAPH_SYSTEM_BUILDER_HPP_
#include <chrono>
#include <memory>
#include <vector>
#include <string>

#include "settings.hpp"
#include "reference_system/sample_management.hpp"

using namespace std::chrono_literals;

template<typename SystemType, typename TimingConfig>
auto create_graph_nodes_underutilized()
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
      rt_nodes::SensorSettings{.node_name = "SensorA",
        .topic_name = "Data",
        .cycle_time = 25ms,
        .number_crunch_limit = 8192,
        .wcet = 900000}));
  nodes.emplace_back(
    std::make_shared<typename SystemType::Sensor>(
      rt_nodes::SensorSettings{.node_name = "SensorB",
        .topic_name = "Data",
        .cycle_time = 41ms,
        .number_crunch_limit = 8192,
        .wcet = 900000}));
  nodes.emplace_back(
    std::make_shared<typename SystemType::Sensor>(
      rt_nodes::SensorSettings{.node_name = "SensorC",
        .topic_name = "Data",
        .cycle_time = 51ms,
        .number_crunch_limit = 8192,
        .wcet = 4800000}));

  nodes.emplace_back(
    std::make_shared<typename SystemType::Transform>(
      rt_nodes::TransformSettings{.node_name = "ProcessorX",
        .input_topic = "Data",
        .output_topic = "Sink",
        .number_crunch_limit = 8192,
        .cycle_time = 0ms,
        .wcet = 1750000}));
  nodes.emplace_back(
    std::make_shared<typename SystemType::Transform>(
      rt_nodes::TransformSettings{.node_name = "ProcessorY",
        .input_topic = "Data",
        .output_topic = "Sink",
        .number_crunch_limit = 8192,
        .cycle_time = 0ms,
        .wcet = 1750000}));

#pragma GCC diagnostic pop

  return nodes;
}

template<typename SystemType, typename TimingConfig>
auto create_graph_nodes()
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
      rt_nodes::SensorSettings{.node_name = "SensorA",
        .topic_name = "Data",
        .cycle_time = 25ms,
        .number_crunch_limit = 8192,
        .wcet = 1900000}));
  nodes.emplace_back(
    std::make_shared<typename SystemType::Sensor>(
      rt_nodes::SensorSettings{.node_name = "SensorB",
        .topic_name = "Data",
        .cycle_time = 41ms,
        .number_crunch_limit = 8192,
        .wcet = 900000}));
  nodes.emplace_back(
    std::make_shared<typename SystemType::Sensor>(
      rt_nodes::SensorSettings{.node_name = "SensorC",
        .topic_name = "Data",
        .cycle_time = 51ms,
        .number_crunch_limit = 8192,
        .wcet = 11900000}));

  nodes.emplace_back(
    std::make_shared<typename SystemType::Transform>(
      rt_nodes::TransformSettings{.node_name = "ProcessorX",
        .input_topic = "Data",
        .output_topic = "Sink",
        .number_crunch_limit = 8192,
        .cycle_time = 0ms,
        .wcet = 1750000}));
  nodes.emplace_back(
    std::make_shared<typename SystemType::Transform>(
      rt_nodes::TransformSettings{.node_name = "ProcessorY",
        .input_topic = "Data",
        .output_topic = "Sink",
        .number_crunch_limit = 8192,
        .cycle_time = 0ms,
        .wcet = 750000}));

#pragma GCC diagnostic pop

  return nodes;
}


template<typename SystemType, typename TimingConfig>
auto create_graph_nodes_overutilized()
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
      rt_nodes::SensorSettings{.node_name = "SensorA",
        .topic_name = "Data",
        .cycle_time = 25ms,
        .number_crunch_limit = 8192,
        .wcet = 2900000}));
  nodes.emplace_back(
    std::make_shared<typename SystemType::Sensor>(
      rt_nodes::SensorSettings{.node_name = "SensorB",
        .topic_name = "Data",
        .cycle_time = 41ms,
        .number_crunch_limit = 8192,
        .wcet = 900000}));
  nodes.emplace_back(
    std::make_shared<typename SystemType::Sensor>(
      rt_nodes::SensorSettings{.node_name = "SensorC",
        .topic_name = "Data",
        .cycle_time = 51ms,
        .number_crunch_limit = 8192,
        .wcet = 11900000}));

  nodes.emplace_back(
    std::make_shared<typename SystemType::Transform>(
      rt_nodes::TransformSettings{.node_name = "ProcessorX",
        .input_topic = "Data",
        .output_topic = "Sink",
        .number_crunch_limit = 8192,
        .cycle_time = 0ms,
        .wcet = 3750000}));
  nodes.emplace_back(
    std::make_shared<typename SystemType::Transform>(
      rt_nodes::TransformSettings{.node_name = "ProcessorY",
        .input_topic = "Data",
        .output_topic = "Sink",
        .number_crunch_limit = 8192,
        .cycle_time = 0ms,
        .wcet = 1750000}));

#pragma GCC diagnostic pop

  return nodes;
}


template<typename SystemType, typename TimingConfig>
auto create_graph_nodes_human_speed()
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
      rt_nodes::SensorSettings{.node_name = "SensorA",
        .topic_name = "Data",
        .cycle_time = 2500ms,
        .number_crunch_limit = 81920,
        .wcet = 290000000}));
  nodes.emplace_back(
    std::make_shared<typename SystemType::Sensor>(
      rt_nodes::SensorSettings{.node_name = "SensorB",
        .topic_name = "Data",
        .cycle_time = 4100ms,
        .number_crunch_limit = 81920,
        .wcet = 90000000}));
  nodes.emplace_back(
    std::make_shared<typename SystemType::Sensor>(
      rt_nodes::SensorSettings{.node_name = "SensorC",
        .topic_name = "Data",
        .cycle_time = 5100ms,
        .number_crunch_limit = 81920,
        .wcet = 1190000000}));

  nodes.emplace_back(
    std::make_shared<typename SystemType::Transform>(
      rt_nodes::TransformSettings{.node_name = "ProcessorX",
        .input_topic = "Data",
        .output_topic = "Sink",
        .number_crunch_limit = 81920,
        .cycle_time = 0ms,
        .wcet = 390000000}));
  nodes.emplace_back(
    std::make_shared<typename SystemType::Transform>(
      rt_nodes::TransformSettings{.node_name = "ProcessorY",
        .input_topic = "Data",
        .output_topic = "Sink",
        .number_crunch_limit = 81920,
        .cycle_time = 0ms,
        .wcet = 190000000}));

#pragma GCC diagnostic pop

  return nodes;
}

#endif  // AUTOWARE_REFERENCE_SYSTEM__SEQUENCES_SYSTEM_BUILDER_HPP_