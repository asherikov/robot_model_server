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
// INTERRUPTION HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include "robot_model_server_ros.hpp"

#include <chrono>
#include <cstdlib>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/string.hpp"

namespace robot_model_server_ros
{

    namespace
    {

        std::chrono::nanoseconds toCoreTime(const builtin_interfaces::msg::Time &time)
        {
            return std::chrono::seconds(time.sec) + std::chrono::nanoseconds(time.nanosec);
        }

        std::vector<geometry_msgs::msg::TransformStamped> toTransformStamped(
                const std::vector<robot_model_server_core::Transform> &transforms)
        {
            std::vector<geometry_msgs::msg::TransformStamped> result;
            result.reserve(transforms.size());
            for (const auto &tf : transforms)
            {
                geometry_msgs::msg::TransformStamped msg;
                msg.header.stamp.sec =
                        static_cast<int32_t>(std::chrono::duration_cast<std::chrono::seconds>(tf.stamp).count());
                msg.header.stamp.nanosec = static_cast<uint32_t>((tf.stamp % std::chrono::seconds(1)).count());
                msg.header.frame_id = tf.frame_id;
                msg.child_frame_id = tf.child_frame_id;
                msg.transform.translation.x = tf.translation.x;
                msg.transform.translation.y = tf.translation.y;
                msg.transform.translation.z = tf.translation.z;
                msg.transform.rotation.x = tf.rotation.x;
                msg.transform.rotation.y = tf.rotation.y;
                msg.transform.rotation.z = tf.rotation.z;
                msg.transform.rotation.w = tf.rotation.w;
                result.push_back(msg);
            }
            return result;
        }

    }  // namespace

    RobotStatePublisher::RobotStatePublisher(const rclcpp::NodeOptions &options)
      : rclcpp::Node("robot_model_server", options)
    {
        const std::string urdf_xml = this->declare_parameter("robot_description", std::string(""));
        if (urdf_xml.empty())
        {
            throw std::runtime_error("robot_description parameter must not be empty");
        }

        description_pub_ =
                this->create_publisher<std_msgs::msg::String>("robot_description", rclcpp::QoS(1).transient_local());

        core_.setupURDF(urdf_xml);

        auto description_msg = std::make_unique<std_msgs::msg::String>();
        description_msg->data = urdf_xml;
        description_pub_->publish(std::move(description_msg)); // NOLINT

        publish_frequency_ = this->declare_parameter("publish_frequency", 20.0);
        if (!robot_model_server_core::RobotModelCore::checkValidPubFreq(publish_frequency_))
        {
            throw std::runtime_error("publish_frequency must be between 0 (exclusive) and 1000");
        }

        frame_prefix_ = this->declare_parameter("frame_prefix", std::string(""));

        ignore_timestamp_ = this->declare_parameter("ignore_timestamp", false);

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        static_tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);

        auto subscriber_options = rclcpp::SubscriptionOptions();
        subscriber_options.qos_overriding_options = rclcpp::QosOverridingOptions::with_default_policies();

        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
                "joint_states",
                rclcpp::SensorDataQoS(),
                [this](const sensor_msgs::msg::JointState::ConstSharedPtr &state) { callbackJointState(state); },
                subscriber_options);

        publishFixedTransforms();
    }

    void RobotStatePublisher::publishTransforms(
            const std::map<std::string, double> &joint_positions,
            const builtin_interfaces::msg::Time &time)
    {
        const auto tf_transforms = core_.getTransforms(joint_positions, toCoreTime(time), frame_prefix_);
        tf_broadcaster_->sendTransform(toTransformStamped(tf_transforms));
    }

    void RobotStatePublisher::publishFixedTransforms()
    {
        const auto now = this->now();
        const auto core_time = std::chrono::nanoseconds(now.nanoseconds());
        const auto tf_transforms = core_.getFixedTransforms(core_time, frame_prefix_);
        static_tf_broadcaster_->sendTransform(toTransformStamped(tf_transforms));
    }

    void RobotStatePublisher::callbackJointState(const sensor_msgs::msg::JointState::ConstSharedPtr &state)
    {
        if (state->name.size() != state->position.size())
        {
            if (state->position.empty())
            {
                const char *first_joint = state->name.empty() ? "<none>" : state->name.at(0).c_str();
                RCLCPP_WARN(
                        get_logger(),
                        "Robot state publisher ignored a JointState message about joint(s) "
                        "\"%s\"(,...) whose position member was empty.",
                        first_joint);
            }
            else
            {
                RCLCPP_ERROR(get_logger(), "Robot state publisher ignored an invalid JointState message");
            }
            return;
        }

        const rclcpp::Time now = this->now();
        if (last_callback_time_.nanoseconds() > now.nanoseconds())
        {
            RCLCPP_WARN(get_logger(), "Moved backwards in time, re-publishing joint transforms!");
            last_publish_time_.clear();
        }
        last_callback_time_ = now;

        rclcpp::Time last_published = now;
        for (const std::string &name : state->name)
        {
            const rclcpp::Time t(last_publish_time_[name]);
            last_published = (t.nanoseconds() < last_published.nanoseconds()) ? t : last_published;
        }

        const rclcpp::Time current_time(state->header.stamp);
        const std::chrono::milliseconds publish_interval_ms =
                std::chrono::milliseconds(static_cast<uint64_t>(1000.0 / publish_frequency_));
        const rclcpp::Time max_publish_time = last_published + rclcpp::Duration(publish_interval_ms);
        if (ignore_timestamp_ || current_time.nanoseconds() >= max_publish_time.nanoseconds())
        {
            std::map<std::string, double> joint_positions;
            for (size_t i = 0; i < state->name.size(); i++)
            {
                joint_positions.emplace(state->name.at(i), state->position.at(i));
            }

            core_.computeMimicJoints(joint_positions);

            publishTransforms(joint_positions, state->header.stamp);

            for (const std::string &name : state->name)
            {
                last_publish_time_[name] = state->header.stamp;
            }
        }
    }

}  // namespace robot_model_server_ros

int main(int argc, char **argv)
{
    try
    {
        rclcpp::init(argc, argv);
        rclcpp::spin(std::make_shared<robot_model_server_ros::RobotStatePublisher>(rclcpp::NodeOptions()));
        rclcpp::shutdown();
        return 0;
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error: " << e.what() << '\n';
        rclcpp::shutdown();
        return EXIT_FAILURE;
    }
}
