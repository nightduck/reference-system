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
#ifndef REFERENCE_SYSTEM__NODES__RCLCPP__SENSOR_HPP_
#define REFERENCE_SYSTEM__NODES__RCLCPP__SENSOR_HPP_
#include <chrono>
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "reference_system/msg_types.hpp"
#include "reference_system/nodes/settings.hpp"
#include "reference_system/number_cruncher.hpp"
#include "reference_system/sample_management.hpp"

namespace nodes
{
namespace rclcpp_system
{

class PeriodicSensor : public rclcpp::Node
{
public:
  explicit PeriodicSensor(const PeriodicSensorSettings & settings)
  : Node(settings.node_name),
    number_crunch_limit_(settings.number_crunch_limit)
  {
    publisher_ = this->create_publisher<message_t>(settings.topic_name, 1);
    timer_ = this->create_wall_timer(
      settings.cycle_time,
      [this] {timer_callback();}, nullptr, {publisher_});
  }

private:
  void timer_callback()
  {

    uint64_t timestamp = now_as_int();
    auto number_cruncher_result = number_cruncher(number_crunch_limit_);

    // TODO: Write this
    // Get the next arrival time of timer_
    int64_t next_call_time = rcutils_atomic_load_int64_t(timer_->get_timer_handle()->impl->next_call_time);
    int64_t period = timer_->get_timer_handle()->impl->period;
    if (sequence_number_ == 0) {
      expected_arrival_time = now_as_int();
    } else if (next_call_time - period != expected_arrival_time) {
      // !!! Dropped a job!!
    } 
    expected_arrival_time += period;

    auto message = publisher_->borrow_loaned_message();
    message.get().size = 0;

    set_sample(
      this->get_name(), sequence_number_++, 0, timestamp,
      message.get());

    publisher_->publish(std::move(message));
  }

private:
  rclcpp::Publisher<message_t>::SharedPtr publisher_;
  uint64_t number_crunch_limit_;
  int64_t expected_arrival_time;
  
  rclcpp::TimerBase::SharedPtr timer_;
  uint32_t sequence_number_ = 0;
};
}  // namespace rclcpp_system
}  // namespace nodes
#endif  // REFERENCE_SYSTEM__NODES__RCLCPP__SENSOR_HPP_
