// Copyright (c) 2008, Willow Garage, Inc.
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
#include <memory>
#include <thread>

#include "gtest/gtest.h"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/version.h"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"

#define EPS 0.01

TEST(TestPublisher, TestTwoLinksFixedJointPrefix)
{
    auto node = rclcpp::Node::make_shared("rsp_test_two_links_fixed_joint_prefix", "test_fixed_joint_prefix");

    const rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
    tf2_ros::Buffer buffer(clock);
#if RCLCPP_VERSION_GTE(29, 0, 0)
    const tf2_ros::TransformListener tfl(
        buffer, tf2_ros::TransformListener::RequiredInterfaces(*node), true);
#else
    const tf2_ros::TransformListener tfl(buffer, node, true);
#endif

    ASSERT_TRUE(buffer.canTransform(
            "my_prefix/link1", "my_prefix/link2", rclcpp::Time(), rclcpp::Duration(std::chrono::seconds(3))));
    ASSERT_FALSE(buffer.canTransform("base_link", "wim_link", rclcpp::Time()));

    const geometry_msgs::msg::TransformStamped t =
            buffer.lookupTransform("my_prefix/link1", "my_prefix/link2", rclcpp::Time());
    EXPECT_NEAR(t.transform.translation.x, 5.0, EPS);
    EXPECT_NEAR(t.transform.translation.y, 0.0, EPS);
    EXPECT_NEAR(t.transform.translation.z, 0.0, EPS);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    rclcpp::init(argc, argv);

    const int res = RUN_ALL_TESTS();

    rclcpp::shutdown();

    return res;
}
