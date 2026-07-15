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
#include <memory>
#include <thread>

#include "gtest/gtest.h"

#include "geometry_msgs/msg/inertia.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/version.h"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

static constexpr double EPS = 0.01;

TEST(TestInertial, TestInertialMarkers)
{
    auto node = rclcpp::Node::make_shared("rsp_test_inertial");

    const rclcpp::QoS latched_qos = rclcpp::QoS(1).transient_local();

    std::vector<visualization_msgs::msg::MarkerArray> received_markers;
    auto marker_sub = node->create_subscription<visualization_msgs::msg::MarkerArray>(
        "/test_inertial/robot_model_server/inertia_visual", latched_qos,
        [&received_markers](const visualization_msgs::msg::MarkerArray::ConstSharedPtr &msg) {
            received_markers.push_back(*msg);
        });

    const rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
    tf2_ros::Buffer buffer(clock);
#if RCLCPP_VERSION_GTE(29, 0, 0)
    const tf2_ros::TransformListener tfl(
        buffer, tf2_ros::TransformListener::RequiredInterfaces(*node), true);
#else
    const tf2_ros::TransformListener tfl(buffer, node, true);
#endif

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    const auto start = std::chrono::steady_clock::now();
    const visualization_msgs::msg::MarkerArray * link_markers = nullptr;
    while (link_markers == nullptr &&
           std::chrono::steady_clock::now() - start < std::chrono::seconds(10))
    {
        executor.spin_some(std::chrono::milliseconds(100));
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        for (const auto & array : received_markers)
        {
            bool has_link1 = false;
            bool has_link2 = false;
            for (const auto & marker : array.markers)
            {
                if (marker.ns == "link1")
                {
                    has_link1 = true;
                }
                if (marker.ns == "link2")
                {
                    has_link2 = true;
                }
            }
            if (has_link1 && has_link2)
            {
                link_markers = &array;
                break;
            }
        }
    }

    ASSERT_NE(link_markers, nullptr);
    const auto & markers = link_markers->markers;

    int link1_idx = -1;
    int link2_idx = -1;
    for (size_t i = 0; i < markers.size(); ++i)
    {
        if (markers.at(i).ns == "link1")
        {
            link1_idx = static_cast<int>(i);
        }
        else if (markers.at(i).ns == "link2")
        {
            link2_idx = static_cast<int>(i);
        }
    }
    ASSERT_NE(link1_idx, -1);
    ASSERT_NE(link2_idx, -1);

    EXPECT_NEAR(markers.at(link1_idx).pose.position.x, 0.0, EPS);
    EXPECT_NEAR(markers.at(link1_idx).pose.position.y, 0.0, EPS);
    EXPECT_NEAR(markers.at(link1_idx).pose.position.z, 0.0, EPS);
    EXPECT_NEAR(markers.at(link1_idx).scale.x, std::sqrt(0.6), EPS);
    EXPECT_NEAR(markers.at(link1_idx).scale.y, std::sqrt(0.6), EPS);
    EXPECT_NEAR(markers.at(link1_idx).scale.z, std::sqrt(0.6), EPS);

    EXPECT_NEAR(markers.at(link2_idx).pose.position.x, 0.5, EPS);
    EXPECT_NEAR(markers.at(link2_idx).pose.position.y, 0.0, EPS);
    EXPECT_NEAR(markers.at(link2_idx).pose.position.z, 0.0, EPS);
    EXPECT_NEAR(markers.at(link2_idx).scale.x, std::sqrt(0.6), EPS);
    EXPECT_NEAR(markers.at(link2_idx).scale.y, std::sqrt(0.6), EPS);
    EXPECT_NEAR(markers.at(link2_idx).scale.z, std::sqrt(0.6), EPS);

    EXPECT_NEAR(markers.at(link1_idx).color.a, 0.5, EPS);
    EXPECT_NEAR(markers.at(link2_idx).color.a, 0.5, EPS);

    EXPECT_NEAR(markers.at(link1_idx).color.r, 0.2, EPS);
    EXPECT_NEAR(markers.at(link1_idx).color.g, 0.8, EPS);
    EXPECT_NEAR(markers.at(link1_idx).color.b, 0.2, EPS);
    EXPECT_NEAR(markers.at(link2_idx).color.r, 0.2, EPS);
    EXPECT_NEAR(markers.at(link2_idx).color.g, 0.8, EPS);
    EXPECT_NEAR(markers.at(link2_idx).color.b, 0.2, EPS);
}

TEST(TestInertial, TestCumulativeInertia)
{
    auto node = rclcpp::Node::make_shared("rsp_test_inertial_cumulative");

    const rclcpp::QoS latched_qos = rclcpp::QoS(1).transient_local();

    std::vector<geometry_msgs::msg::Inertia> received_inertia;
    auto inertia_sub = node->create_subscription<geometry_msgs::msg::Inertia>(
        "/test_inertial/robot_model_server/cumulative_inertia", latched_qos,
        [&received_inertia](const geometry_msgs::msg::Inertia::ConstSharedPtr &msg) {
            received_inertia.push_back(*msg);
        });

    const rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
    tf2_ros::Buffer buffer(clock);
#if RCLCPP_VERSION_GTE(29, 0, 0)
    const tf2_ros::TransformListener tfl(
        buffer, tf2_ros::TransformListener::RequiredInterfaces(*node), true);
#else
    const tf2_ros::TransformListener tfl(buffer, node, true);
#endif

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    const auto start = std::chrono::steady_clock::now();
    while (received_inertia.empty() &&
           std::chrono::steady_clock::now() - start < std::chrono::seconds(10))
    {
        executor.spin_some(std::chrono::milliseconds(100));
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    ASSERT_FALSE(received_inertia.empty());
    const auto & inertia = received_inertia.front();

    EXPECT_NEAR(inertia.m, 3.0, EPS);
    EXPECT_NEAR(inertia.com.x, 1.0, EPS);
    EXPECT_NEAR(inertia.com.y, 0.0, EPS);
    EXPECT_NEAR(inertia.com.z, 0.0, EPS);
    EXPECT_NEAR(inertia.ixx, 0.3, EPS);
    EXPECT_NEAR(inertia.ixy, 0.0, EPS);
    EXPECT_NEAR(inertia.ixz, 0.0, EPS);
    EXPECT_NEAR(inertia.iyy, 1.8, EPS);
    EXPECT_NEAR(inertia.iyz, 0.0, EPS);
    EXPECT_NEAR(inertia.izz, 1.8, EPS);

    ASSERT_TRUE(buffer.canTransform(
        "link1", "cumulative_center_of_mass",
        rclcpp::Time(), rclcpp::Duration(std::chrono::seconds(5))));

    const auto com_tf = buffer.lookupTransform("link1", "cumulative_center_of_mass", rclcpp::Time());
    EXPECT_NEAR(com_tf.transform.translation.x, 1.0, EPS);
    EXPECT_NEAR(com_tf.transform.translation.y, 0.0, EPS);
    EXPECT_NEAR(com_tf.transform.translation.z, 0.0, EPS);
    EXPECT_NEAR(com_tf.transform.rotation.x, 0.0, EPS);
    EXPECT_NEAR(com_tf.transform.rotation.y, 0.0, EPS);
    EXPECT_NEAR(com_tf.transform.rotation.z, 0.0, EPS);
    EXPECT_NEAR(com_tf.transform.rotation.w, 1.0, EPS);
}

TEST(TestInertial, TestCumulativeInertiaVisual)
{
    auto node = rclcpp::Node::make_shared("rsp_test_inertial_visual");

    const rclcpp::QoS latched_qos = rclcpp::QoS(1).transient_local();

    std::vector<visualization_msgs::msg::MarkerArray> received_markers;
    auto marker_sub = node->create_subscription<visualization_msgs::msg::MarkerArray>(
        "/test_inertial/robot_model_server/inertia_visual", latched_qos,
        [&received_markers](const visualization_msgs::msg::MarkerArray::ConstSharedPtr &msg) {
            received_markers.push_back(*msg);
        });

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    const auto start = std::chrono::steady_clock::now();
    bool found_cumulative = false;
    visualization_msgs::msg::Marker cumulative_marker;
    while (!found_cumulative &&
           std::chrono::steady_clock::now() - start < std::chrono::seconds(10))
    {
        executor.spin_some(std::chrono::milliseconds(100));
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        for (const auto & array : received_markers)
        {
            for (const auto & marker : array.markers)
            {
                if (marker.ns == "cumulative_inertial")
                {
                    cumulative_marker = marker;
                    found_cumulative = true;
                    break;
                }
            }
            if (found_cumulative)
            {
                break;
            }
        }
    }

    ASSERT_TRUE(found_cumulative);

    EXPECT_EQ(cumulative_marker.header.frame_id, "link1");
    EXPECT_NEAR(cumulative_marker.pose.position.x, 1.0, EPS);
    EXPECT_NEAR(cumulative_marker.pose.position.y, 0.0, EPS);
    EXPECT_NEAR(cumulative_marker.pose.position.z, 0.0, EPS);
    EXPECT_NEAR(cumulative_marker.scale.x, 1.0, EPS);
    EXPECT_NEAR(cumulative_marker.scale.y, std::sqrt(0.6) / std::sqrt(6.6), EPS);
    EXPECT_NEAR(cumulative_marker.scale.z, std::sqrt(0.6) / std::sqrt(6.6), EPS);

    EXPECT_NEAR(cumulative_marker.color.a, 0.5, EPS);
    EXPECT_NEAR(cumulative_marker.color.r, 0.2, EPS);
    EXPECT_NEAR(cumulative_marker.color.g, 0.2, EPS);
    EXPECT_NEAR(cumulative_marker.color.b, 0.8, EPS);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    rclcpp::init(argc, argv);

    const int res = RUN_ALL_TESTS();

    rclcpp::shutdown();

    return res;
}