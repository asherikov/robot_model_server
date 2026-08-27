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

static std::string loadUrdf()
{
    const char *share_dir = std::getenv("FFW_DESCRIPTION_URDF_DIR");
    if (!share_dir)
    {
        throw std::runtime_error("FFW_DESCRIPTION_URDF_DIR not set");
    }
    const std::string path = std::string(share_dir) + "/ffw_bg2_follower.urdf";
    std::ifstream f(path);
    if (!f.is_open())
    {
        throw std::runtime_error("Cannot open URDF: " + path);
    }
    std::ostringstream ss;
    ss << f.rdbuf();
    return ss.str();
}

static std::vector<std::string> parseMovableJointNames(const std::string &urdf)
{
    std::vector<std::string> names;
    const std::regex re(R"re(<joint\s+name="([^"]+)"\s+type="(revolute|prismatic|continuous)")re");
    for (std::sregex_iterator it(urdf.begin(), urdf.end(), re), end; it != end; ++it)
    {
        names.push_back((*it)[1]);
    }
    return names;
}

class CrossVerifyTest : public ::testing::Test
{
protected:
    static void SetUpTestSuite()
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

    static void TearDownTestSuite()
    {
        pub_.reset();
        tfl_.reset();
        buffer_.reset();
        node_.reset();
    }

    void verify(const std::string &source, const std::string &target,
                const std::vector<std::string> &names, const std::vector<double> &positions)
    {
        robot_model_server::Model model;
        robot_model_server::Model::Parameters params;
        model.initialize(urdf_, params);

        const auto core_tf = model.getTransform(source, target, names, positions);

        sensor_msgs::msg::JointState js_msg;
        js_msg.header.stamp = node_->now();
        for (size_t i = 0; i < names.size(); ++i)
        {
            js_msg.name.push_back(names.at(i));
            js_msg.position.push_back(positions.at(i));
        }
        pub_->publish(js_msg);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));

        ASSERT_TRUE(buffer_->canTransform(source, target, rclcpp::Time(), rclcpp::Duration::from_seconds(2.0)))
                << "tf lookup failed for " << source << " -> " << target;
        const auto tf_msg = buffer_->lookupTransform(source, target, rclcpp::Time());

        EXPECT_NEAR(core_tf.transform.translation().x(), tf_msg.transform.translation.x, EPS)
                << source << " -> " << target << " translation.x";
        EXPECT_NEAR(core_tf.transform.translation().y(), tf_msg.transform.translation.y, EPS)
                << source << " -> " << target << " translation.y";
        EXPECT_NEAR(core_tf.transform.translation().z(), tf_msg.transform.translation.z, EPS)
                << source << " -> " << target << " translation.z";

        const Eigen::Quaterniond core_quat(core_tf.transform.linear());
        const double dot = std::abs(core_quat.x() * tf_msg.transform.rotation.x
                                    + core_quat.y() * tf_msg.transform.rotation.y
                                    + core_quat.z() * tf_msg.transform.rotation.z
                                    + core_quat.w() * tf_msg.transform.rotation.w);
        EXPECT_GT(dot, 1.0 - EPS) << source << " -> " << target << " rotation";
    }

    static std::string urdf_;
    static rclcpp::Node::SharedPtr node_;
    static std::shared_ptr<tf2_ros::Buffer> buffer_;
    static std::shared_ptr<tf2_ros::TransformListener> tfl_;
    static rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_;
};

std::string CrossVerifyTest::urdf_;
rclcpp::Node::SharedPtr CrossVerifyTest::node_;
std::shared_ptr<tf2_ros::Buffer> CrossVerifyTest::buffer_;
std::shared_ptr<tf2_ros::TransformListener> CrossVerifyTest::tfl_;
rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr CrossVerifyTest::pub_;

TEST_F(CrossVerifyTest, FixedOnlyChain)
{
    verify("world", "base_link", {}, {});
}

TEST_F(CrossVerifyTest, SinglePrismaticJoint)
{
    verify("base_link", "arm_base_link", {"lift_joint"}, {0.1});
}

TEST_F(CrossVerifyTest, SingleRevoluteJoint)
{
    verify("arm_base_link", "arm_l_link1", {"arm_l_joint1"}, {0.5});
}

TEST_F(CrossVerifyTest, TwoRevoluteJoints)
{
    verify("arm_base_link", "arm_l_link2", {"arm_l_joint1", "arm_l_joint2"}, {0.3, -0.4});
}

TEST_F(CrossVerifyTest, FullArmChain)
{
    const std::vector<std::string> names = {
            "arm_l_joint1", "arm_l_joint2", "arm_l_joint3",
            "arm_l_joint4", "arm_l_joint5", "arm_l_joint6", "arm_l_joint7"};
    const std::vector<double> positions = {0.1, -0.2, 0.3, 0.4, -0.1, 0.2, -0.3};
    verify("arm_base_link", "arm_l_link7", names, positions);
}

TEST_F(CrossVerifyTest, FixedPlusMovableChain)
{
    verify("arm_l_link7", "camera_l_link", {}, {});
}

TEST_F(CrossVerifyTest, CrossBranchChain)
{
    const std::vector<std::string> names = {
            "arm_l_joint1", "arm_l_joint2", "arm_l_joint3",
            "arm_l_joint4", "arm_l_joint5", "arm_l_joint6", "arm_l_joint7",
            "arm_r_joint1", "arm_r_joint2", "arm_r_joint3",
            "arm_r_joint4", "arm_r_joint5", "arm_r_joint6", "arm_r_joint7"};
    const std::vector<double> positions = {
            0.1, -0.2, 0.3, 0.4, -0.1, 0.2, -0.3,
            -0.1, 0.2, -0.3, -0.4, 0.1, -0.2, 0.3};
    verify("arm_l_link1", "arm_r_link1", names, positions);
}

TEST_F(CrossVerifyTest, MimicJointChain)
{
    verify("arm_l_link7", "gripper_l_rh_p12_rn_l2",
           {"gripper_l_joint1"}, {0.5});
}

TEST_F(CrossVerifyTest, MimicJointChainWithSource)
{
    verify("arm_l_link7", "gripper_l_rh_p12_rn_r2",
           {"gripper_l_joint1"}, {0.5});
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    rclcpp::init(argc, argv);

    const int res = RUN_ALL_TESTS();

    rclcpp::shutdown();

    return res;
}
