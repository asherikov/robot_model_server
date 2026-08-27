// Copyright (c) 2024, Willow Garage, Inc.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the copyright holder nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <chrono>
#include <cmath>
#include <fstream>
#include <memory>
#include <regex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "gtest/gtest.h"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/version.h"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"

#include "robot_model_server/robot_model_server.hpp"

static constexpr double EPS = 0.01;

namespace
{
std::string loadUrdf()
{
    const char *share_dir = std::getenv("PR2_DESCRIPTION_URDF_DIR"); // NOLINT(concurrency-mt-unsafe)
    if (share_dir == nullptr)
    {
        throw std::runtime_error("PR2_DESCRIPTION_URDF_DIR not set");
    }
    const std::string path = std::string(share_dir) + "/robot.xml";
    std::ifstream f(path);
    if (!f.is_open())
    {
        throw std::runtime_error("Cannot open URDF: " + path);
    }
    std::ostringstream ss;
    ss << f.rdbuf();
    return ss.str();
}

std::vector<std::string> parseMovableJointNames(const std::string &urdf)
{
    std::vector<std::string> names;
    const std::regex re(R"re(<joint\s+name="([^"]+)"\s+type="(revolute|prismatic|continuous)")re");
    for (std::sregex_iterator it(urdf.begin(), urdf.end(), re), end; it != end; ++it)
    {
        names.push_back(it->str(1));
    }
    return names;
}
} // namespace

namespace
{
class CrossVerifyTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        urdf_ = loadUrdf();
        node_ = rclcpp::Node::make_shared("cross_verify", "test_cross_verify");

        const rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
        buffer_ = std::make_shared<tf2_ros::Buffer>(clock);
#if RCLCPP_VERSION_GTE(29, 0, 0)
        tfl_ = std::make_shared<tf2_ros::TransformListener>(
                *buffer_, tf2_ros::TransformListener::RequiredInterfaces(*node_), true);
#else
        tfl_ = std::make_shared<tf2_ros::TransformListener>(*buffer_, node_.get(), true);
#endif
        pub_ = node_->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

        const auto joint_names = parseMovableJointNames(urdf_);

        sensor_msgs::msg::JointState js_msg;
        for (const auto &name : joint_names)
        {
            js_msg.name.push_back(name);
            js_msg.position.push_back(0.0);
        }

        for (unsigned int i = 0; i < 200; ++i)
        {
            js_msg.header.stamp = node_->now();
            pub_->publish(js_msg);
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    }

    void TearDown() override
    {
        pub_.reset();
        tfl_.reset();
        buffer_.reset();
        node_.reset();
    }

    void verify(const std::string &target, const std::string &source,
                const std::vector<std::string> &names, const std::vector<double> &positions)
    {
        robot_model_server::Model model;
        const robot_model_server::Model::Parameters params;
        model.initialize(urdf_, params);

        const auto core_tf = model.getTransform(target, source, names, positions);

        sensor_msgs::msg::JointState js_msg;
        js_msg.header.stamp = node_->now();
        for (size_t i = 0; i < names.size(); ++i)
        {
            js_msg.name.push_back(names.at(i));
            js_msg.position.push_back(positions.at(i));
        }
        pub_->publish(js_msg);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));

        ASSERT_TRUE(buffer_->canTransform(target, source, rclcpp::Time(), rclcpp::Duration::from_seconds(2.0)))
                << "tf lookup failed for " << source << " -> " << target;
        const auto tf_msg = buffer_->lookupTransform(target, source, rclcpp::Time());

        EXPECT_NEAR(core_tf.transform.translation().x(), tf_msg.transform.translation.x, EPS)
                << source << " -> " << target << " translation.x";
        EXPECT_NEAR(core_tf.transform.translation().y(), tf_msg.transform.translation.y, EPS)
                << source << " -> " << target << " translation.y";
        EXPECT_NEAR(core_tf.transform.translation().z(), tf_msg.transform.translation.z, EPS)
                << source << " -> " << target << " translation.z";

        const Eigen::Quaterniond core_quat(core_tf.transform.linear());
        const double dot = std::abs((core_quat.x() * tf_msg.transform.rotation.x)
                                    + (core_quat.y() * tf_msg.transform.rotation.y)
                                    + (core_quat.z() * tf_msg.transform.rotation.z)
                                    + (core_quat.w() * tf_msg.transform.rotation.w));
        EXPECT_GT(dot, 1.0 - EPS) << source << " -> " << target << " rotation";
    }

    std::string urdf_;
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<tf2_ros::Buffer> buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tfl_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_;
};
} // namespace

TEST_F(CrossVerifyTest, FixedOnlyChain)
{
    verify("base_link", "base_footprint", {}, {});
}

TEST_F(CrossVerifyTest, SinglePrismaticJoint)
{
    verify("torso_lift_link", "base_link", {"torso_lift_joint"}, {0.1});
}

TEST_F(CrossVerifyTest, SingleRevoluteJoint)
{
    verify("l_shoulder_pan_link", "torso_lift_link", {"l_shoulder_pan_joint"}, {0.5});
}

TEST_F(CrossVerifyTest, TwoRevoluteJoints)
{
    verify("l_shoulder_lift_link", "torso_lift_link",
           {"l_shoulder_pan_joint", "l_shoulder_lift_joint"}, {0.3, -0.4});
}

TEST_F(CrossVerifyTest, FullArmChain)
{
    const std::vector<std::string> names = {
            "l_shoulder_pan_joint", "l_shoulder_lift_joint", "l_upper_arm_roll_joint",
            "l_elbow_flex_joint", "l_forearm_roll_joint", "l_wrist_flex_joint", "l_wrist_roll_joint"};
    const std::vector<double> positions = {0.1, -0.2, 0.3, 0.4, -0.1, 0.2, -0.3};
    verify("l_wrist_roll_link", "torso_lift_link", names, positions);
}

TEST_F(CrossVerifyTest, FixedPlusMovableChain)
{
    verify("l_gripper_palm_link", "l_wrist_roll_link", {}, {});
}

TEST_F(CrossVerifyTest, CrossBranchChain)
{
    const std::vector<std::string> names = {
            "l_shoulder_pan_joint", "l_shoulder_lift_joint", "l_upper_arm_roll_joint",
            "l_elbow_flex_joint", "l_forearm_roll_joint", "l_wrist_flex_joint", "l_wrist_roll_joint",
            "r_shoulder_pan_joint", "r_shoulder_lift_joint", "r_upper_arm_roll_joint",
            "r_elbow_flex_joint", "r_forearm_roll_joint", "r_wrist_flex_joint", "r_wrist_roll_joint"};
    const std::vector<double> positions = {
            0.1, -0.2, 0.3, 0.4, -0.1, 0.2, -0.3,
            -0.1, 0.2, -0.3, -0.4, 0.1, -0.2, 0.3};
    verify("r_shoulder_pan_link", "l_shoulder_pan_link", names, positions);
}

TEST_F(CrossVerifyTest, GripperJointChain)
{
    verify("l_gripper_l_finger_tip_link", "l_gripper_palm_link",
           {"l_gripper_l_finger_joint"}, {0.5});
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    rclcpp::init(argc, argv);

    const int res = RUN_ALL_TESTS();

    rclcpp::shutdown();

    return res;
}
