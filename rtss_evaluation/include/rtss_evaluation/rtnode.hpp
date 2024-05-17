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
#ifndef REFERENCE_SYSTEM__SYSTEM__TYPE__RT_NODE_HPP_
#define REFERENCE_SYSTEM__SYSTEM__TYPE__RT_NODE_HPP_

#include <chrono>
#include <string>

#include "rclcpp/rclcpp.hpp"


namespace rt_nodes
{
namespace rt_system
{
class RTNode : public rclcpp::Node {
public:
  explicit RTNode(const std::string & name) : Node(name) {}

  virtual uint32_t
  get_dropped_jobs() const = 0;

  virtual uint32_t
  get_completed_jobs() const = 0;

  virtual uint64_t
  get_first_job() const = 0;

  virtual uint64_t
  get_last_job() const = 0;

  virtual uint64_t
  get_timer_period() const = 0;
};

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
  
}  // namespace rt_system
}  // namespace rt_nodes

#endif  // REFERENCE_SYSTEM__SYSTEM__TYPE__RT_NODE_HPP_