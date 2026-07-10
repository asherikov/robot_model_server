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

#ifndef ROBOT_MODEL_SERVER_ROS_ROBOT_MODEL_SERVER_ROS_HPP_
#define ROBOT_MODEL_SERVER_ROS_ROBOT_MODEL_SERVER_ROS_HPP_

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_model_server_core/robot_model_core.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/static_transform_broadcaster.hpp"
#include "tf2_ros/transform_broadcaster.hpp"

namespace robot_model_server_ros
{

    class RobotStatePublisher : public rclcpp::Node
    {
    public:
        explicit RobotStatePublisher(const rclcpp::NodeOptions &options);

    protected:
        void publishTransforms(
                const std::map<std::string, double> &joint_positions,
                const builtin_interfaces::msg::Time &time);

        void publishFixedTransforms();

        void callbackJointState(const sensor_msgs::msg::JointState::ConstSharedPtr &state);

        robot_model_server_core::RobotModelCore core_;

        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
        std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr description_pub_;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
        rclcpp::Time last_callback_time_;
        std::map<std::string, builtin_interfaces::msg::Time> last_publish_time_;
        double publish_frequency_;
        bool ignore_timestamp_;
        std::string frame_prefix_;
    };

}  // namespace robot_model_server_ros

#endif  // ROBOT_MODEL_SERVER_ROS_ROBOT_MODEL_SERVER_ROS_HPP_
