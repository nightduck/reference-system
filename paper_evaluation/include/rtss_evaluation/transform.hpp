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
#ifndef REFERENCE_SYSTEM__NODES__RCLCPP__TRANSFORM_HPP_
#define REFERENCE_SYSTEM__NODES__RCLCPP__TRANSFORM_HPP_
#include <chrono>
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "settings.hpp"
#include "rtnode.hpp"
#include "reference_system/msg_types.hpp"
#include "reference_system/sample_management.hpp"

namespace rt_nodes
{
namespace rt_system
{

class Transform : public RTNode
{
public:
  explicit Transform(const TransformSettings & settings)
  : RTNode(settings.node_name),
    number_crunch_limit_(settings.number_crunch_limit)
  {
    publisher_ = this->create_publisher<message_t>(settings.output_topic, 1);
    subscription_ = this->create_subscription<message_t>(
      settings.input_topic, 1,
    #ifdef RCLCPP_EXPERIMENTAL_PUBLISHER_HINTS
      [this](const message_t::SharedPtr msg) {input_callback(msg);},
      {publisher_});
    #else
      [this](const message_t::SharedPtr msg) {input_callback(msg);});
    #endif
    period = settings.cycle_time.count();
    wcet = settings.wcet;
  }

  uint32_t
  get_dropped_jobs() const override
  {
    return dropped_jobs_;
  }

  uint32_t
  get_completed_jobs() const override
  {
    return sequence_number_;
  }

  uint32_t
  get_deadline_overruns() const override
  {
    return deadline_overruns_;
  }

  uint64_t
  get_first_job() const override
  {
    return first_job;
  }

  uint64_t
  get_last_job() const override
  {
    return last_job;
  }

  uint64_t
  get_timer_period() const override
  {
    return period;
  }

private:
  void input_callback(const message_t::SharedPtr input_message)
  {
    uint64_t timestamp = now_as_int();

    // std::cout << timestamp << ": " << this->get_name() << " received message " << input_sequence_number_ << std::endl;

    // // Get the next arrival time of timer_, and determine if a job was dropped
    // uint32_t missed_jobs = 0;
    // if (sequence_number_ == 0) {
    //   first_job = timestamp;
    //   expected_arrival_time = timestamp;
    // } else if (timestamp > expected_arrival_time) {
    //   // !!! Dropped a job!!
    //   // std::cout << "Dropped a job! Expected arrival time: " << expected_arrival_time <<
    //   //   " next arrival time: " << next_arrival_time << " period: " << period << std::endl;
    //   missed_jobs = ((timestamp - expected_arrival_time) / period);
    //   dropped_jobs_ += missed_jobs;
    // } else {
    //   // If we are early, we should update the expected arrival time
    //   expected_arrival_time = std::min(expected_arrival_time, timestamp);
    // }
    last_job = timestamp;
    // expected_arrival_time += period;

    auto number_cruncher_result = number_cruncher(number_crunch_limit_, timestamp + wcet);

    auto output_message = publisher_->borrow_loaned_message();
    output_message.get().size = 0;
    merge_history_into_sample(output_message.get(), input_message);

    uint32_t missed_samples = get_missed_samples_and_update_seq_nr(
      input_message, input_sequence_number_);

    set_sample(
      this->get_name(), sequence_number_++, missed_samples, timestamp,
      output_message.get());

    // use result so that it is not optimizied away by some clever compiler
    output_message.get().data[0] = number_cruncher_result;
    publisher_->publish(std::move(output_message));
  }

private:
  rclcpp::Publisher<message_t>::SharedPtr publisher_;
  rclcpp::Subscription<message_t>::SharedPtr subscription_;
  uint64_t number_crunch_limit_;
  uint64_t expected_arrival_time;

  uint32_t sequence_number_ = 0;
  uint32_t input_sequence_number_ = 0;
  uint32_t dropped_jobs_ = 0;
  uint32_t deadline_overruns_ = 0;
  uint64_t first_job = UINT64_MAX;
  uint64_t last_job = 0;
  uint64_t period = 0;
  uint64_t wcet = 0;
};
}  // namespace rclcpp_system
}  // namespace nodes
#endif  // REFERENCE_SYSTEM__NODES__RCLCPP__TRANSFORM_HPP_
