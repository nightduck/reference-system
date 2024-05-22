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
#ifndef REFERENCE_SYSTEM__NODES__RCLCPP__PERIODIC_SENSOR_HPP_
#define REFERENCE_SYSTEM__NODES__RCLCPP__PERIODIC_SENSOR_HPP_
#include <chrono>
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "settings.hpp"
#include "rtnode.hpp"
#include "reference_system/msg_types.hpp"
#include "reference_system/sample_management.hpp"

// #include "rcl/timer.h"

namespace rt_nodes
{
namespace rt_system
{

class Sensor : public RTNode
{
public:
  explicit Sensor(const SensorSettings & settings)
  : RTNode(settings.node_name),
    number_crunch_limit_(settings.number_crunch_limit)
  {
    publisher_ = this->create_publisher<message_t>(settings.topic_name, 1);
    timer_ = this->create_wall_timer(
      settings.cycle_time,
      [this] {timer_callback();}, nullptr, {publisher_});
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
  void timer_callback()
  {
    uint64_t timestamp = now_as_int();
    std::chrono::nanoseconds time_left = timer_->time_until_trigger();

    // Get the next arrival time of timer_, and determine if a job was dropped
    int64_t next_arrival_time = timer_->get_arrival_time();
    uint32_t missed_jobs = 0;
    if (sequence_number_ == 0) {
      first_job = timestamp;
    } else if (next_arrival_time - period > expected_arrival_time) {
      // !!! Dropped a job!!
      // std::cout << "Dropped a job! Expected arrival time: " << expected_arrival_time <<
      //   " next arrival time: " << next_arrival_time << " period: " << period << std::endl;
      missed_jobs = ((next_arrival_time - expected_arrival_time) / period) - 1;
      dropped_jobs_ += missed_jobs;
      last_job = timestamp;
      expected_arrival_time = next_arrival_time;
      return;
    }
    last_job = timestamp;
    expected_arrival_time = next_arrival_time;
    
    auto number_cruncher_result = number_cruncher(number_crunch_limit_, timestamp + wcet);

    // If timer is ready, then arrival time (eg our deadline) has passed again
    if (now_as_int() - timestamp > time_left.count()) {
      // std::cout << "Dropped a job! Expected arrival time: " << expected_arrival_time <<
      //   " next arrival time: " << next_arrival_time << " period: " << period << std::endl;
      deadline_overruns_ += 1;
      return;
    }

    // auto message = publisher_->borrow_loaned_message();
    message_t message;
    message.size = 0;

    set_sample(
      this->get_name(), sequence_number_++, missed_jobs, timestamp,
      message);

    // std::cout << timestamp << ": " << this->get_name() << " published message " << sequence_number_ 
    //   << std::endl;
    publisher_->publish(message);
  }

  // int64_t
  // get_period()
  // {
  //   // int64_t period;
  //   // rcl_ret_t ret = rcl_timer_get_period(timer_->get_timer_handle().get(), &period);
  //   // if (ret != RCL_RET_OK) {
  //   //   rclcpp::exceptions::throw_from_rcl_error(ret, "Failed to get timer period");
  //   // }
  //   // return period;
  //   return 5;
  // }

  // int64_t
  // get_arrival_time()
  // {
  //   // int64_t time;
  //   // rcl_ret_t ret = rcl_timer_get_next_call_time(timer_->get_timer_handle().get(), &time);
  //   // if (ret != RCL_RET_OK) {
  //   //   rclcpp::exceptions::throw_from_rcl_error(ret, "Failed to get timer arrival time");
  //   // }
  //   // return time;
  //   return 5;
  // }

private:
  rclcpp::Publisher<message_t>::SharedPtr publisher_;
  uint64_t number_crunch_limit_;
  int64_t expected_arrival_time;
  
  rclcpp::TimerBase::SharedPtr timer_;
  uint32_t sequence_number_ = 0;
  uint32_t dropped_jobs_ = 0;
  uint32_t deadline_overruns_ = 0;
  uint64_t first_job = UINT64_MAX;
  uint64_t last_job = 0;
  uint64_t period = 0;
  uint64_t wcet = 0;
};
}  // namespace rclcpp_system
}  // namespace nodes
#endif  // REFERENCE_SYSTEM__NODES__RCLCPP__PERIODIC_SENSOR_HPP_
