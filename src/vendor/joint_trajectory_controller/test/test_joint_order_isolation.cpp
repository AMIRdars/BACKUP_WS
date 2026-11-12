// Copyright 2026 AMIR workspace contributors
// SPDX-License-Identifier: Apache-2.0
#include <atomic>
#include <thread>
#include <gtest/gtest.h>
#include "joint_trajectory_controller/joint_trajectory_controller.hpp"

class SortProbe : public joint_trajectory_controller::JointTrajectoryController
{
public:
  SortProbe()
  {
    dof_ = 5;
    params_.joints = {"Joint_1", "Joint_2", "Joint_3", "Joint_4", "Joint_5"};
  }
  using JointTrajectoryController::sort_to_local_joint_order;
};

TEST(JointOrderIsolation, ConcurrentRobotCommandsStayLocal)
{
  SortProbe leader, follower;
  std::atomic<int> ready{0};
  std::atomic<bool> failed{false};
  auto run = [&](SortProbe & controller, double offset) {
    auto msg = std::make_shared<trajectory_msgs::msg::JointTrajectory>();
    msg->joint_names = {"Joint_5", "Joint_4", "Joint_3", "Joint_2", "Joint_1"};
    msg->points.resize(2);
    ready.fetch_add(1);
    while (ready.load() < 2) { std::this_thread::yield(); }
    for (int iteration = 0; iteration < 200000 && !failed.load(); ++iteration) {
      for (auto & point : msg->points) {
        point.positions = {offset+5, offset+4, offset+3, offset+2, offset+1};
      }
      controller.sort_to_local_joint_order(msg);
      for (const auto & point : msg->points) {
        for (size_t j = 0; j < 5; ++j) {
          if (point.positions[j] != offset+static_cast<double>(j)+1) {
            failed.store(true);
          }
        }
      }
    }
  };
  std::thread a(run, std::ref(leader), 0.0);
  std::thread b(run, std::ref(follower), 100.0);
  a.join(); b.join();
  EXPECT_FALSE(failed.load()) << "A robot received another controller instance's joint data";
}
