// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
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

#include "gtest/gtest.h"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>

#include <nav2_costmap_2d/costmap_2d.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>

#include "nav2_mppi_controller/controller.hpp"

#include "utils/utils.hpp"
#include "nav2_ros_common/tf2_factories.hpp"

// Tests basic transition from configure->active->process->deactivate->cleanup

TEST(ControllerStateTransitionTest, ControllerNotFail)
{
  const bool visualize = true;
  TestCostmapSettings costmap_settings{};

  // Node Options
  rclcpp::NodeOptions options;
  std::vector<rclcpp::Parameter> params;
  setUpControllerParams(visualize, params);
  options.parameter_overrides(params);

  auto node = getDummyNode(options);
  node->declare_parameter("publish_optimal_trajectory", true);
  auto tf_buffer = nav2::create_transform_buffer(node);
  auto costmap_ros = getDummyCostmapRos(costmap_settings);
  costmap_ros->setRobotFootprint(getDummySquareFootprint(0.01));

  auto controller = getDummyController(node, tf_buffer, costmap_ros);

  TestPose start_pose = costmap_settings.getCenterPose();
  const double path_step = costmap_settings.resolution;
  TestPathSettings path_settings{start_pose, 8, path_step, path_step};

  // evalControl args
  auto pose = getDummyPointStamped(node, start_pose);
  auto velocity = getDummyTwist();
  auto path = getIncrementalDummyPath(node, path_settings);
  path.header.frame_id = costmap_ros->getGlobalFrameID();
  pose.header.frame_id = costmap_ros->getGlobalFrameID();

  controller->newPathReceived(path);
  nav_msgs::msg::Path transformed_global_plan;
  geometry_msgs::msg::PoseStamped goal;
  EXPECT_NO_THROW(controller->computeVelocityCommands(pose, velocity, {}, transformed_global_plan,
    goal));

  controller->setSpeedLimit(0.5, true);
  controller->setSpeedLimit(0.5, false);
  controller->setSpeedLimit(1.0, true);
  controller->deactivate();
  controller->cleanup();
}

class LifecycleControllerTester : public nav2_mppi_controller::MPPIController
{
public:
  unsigned int batchSize() const
  {
    return optimizer_.getSettings().batch_size;
  }
};

TEST(ControllerLifecycle, ReconfigureGpuPrerequisite)
{
  rclcpp::NodeOptions options;
  std::vector<rclcpp::Parameter> params;
  setUpOptimizerParams(TestOptimizerSettings{8, 8, 1, 10.0, "diff_drive", false}, {}, params);
  params.emplace_back("dummy.regenerate_noises", true);
  options.parameter_overrides(params);
  auto node = getDummyNode(options, "validator_controller_node");
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "validator_controller_costmap", "", true);
  ASSERT_EQ(costmap->configure().label(), "inactive");
  auto tf_buffer = nav2::create_transform_buffer(node);
  auto controller = std::make_unique<LifecycleControllerTester>();

  testing::internal::CaptureStderr();
  for (int cycle = 0; cycle < 3; ++cycle) {
    EXPECT_NO_THROW(controller->configure(node, "dummy", tf_buffer, costmap));
    EXPECT_EQ(controller->batchSize(), 8u);
    EXPECT_NO_THROW(controller->activate());
    EXPECT_TRUE(node->set_parameter(rclcpp::Parameter("dummy.batch_size", 9)).successful);
    EXPECT_EQ(controller->batchSize(), 9u);
    EXPECT_NO_THROW(controller->reset());
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = costmap->getGlobalFrameID();
    pose.pose.position.x = 2.0;
    pose.pose.position.y = 2.0;
    pose.pose.orientation.w = 1.0;
    auto goal = pose;
    goal.pose.position.x += 0.2;
    nav_msgs::msg::Path path;
    path.header = pose.header;
    path.poses = {pose, goal};
    geometry_msgs::msg::Twist velocity;
    EXPECT_NO_THROW(controller->computeVelocityCommands(pose, velocity, nullptr, path, goal));
    EXPECT_NO_THROW(controller->deactivate());
    EXPECT_NO_THROW(controller->cleanup());
    // cleanup destroys ParametersHandler and unregisters ROS callbacks. Its old
    // references must not update/reset the optimizer while it is shut down.
    EXPECT_TRUE(node->set_parameter(rclcpp::Parameter("dummy.batch_size", 8)).successful);
    EXPECT_EQ(controller->batchSize(), 9u);
  }
  controller.reset();
  // Updating the surviving node must not reach the destroyed controller either.
  EXPECT_TRUE(node->set_parameter(rclcpp::Parameter("dummy.batch_size", 10)).successful);
  const auto diagnostics = testing::internal::GetCapturedStderr();
  EXPECT_EQ(diagnostics.find("Attempting to unload library"), std::string::npos) << diagnostics;
  EXPECT_EQ(diagnostics.find("LifecycleNode is not shut down"), std::string::npos) << diagnostics;

  EXPECT_EQ(costmap->cleanup().label(), "unconfigured");
  EXPECT_EQ(costmap->shutdown().label(), "finalized");
  EXPECT_EQ(node->shutdown().label(), "finalized");
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  rclcpp::init(0, nullptr);

  int result = RUN_ALL_TESTS();

  rclcpp::shutdown();

  return result;
}
