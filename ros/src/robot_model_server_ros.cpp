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
#include <fstream>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Geometry>

#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/inertia.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "std_msgs/msg/string.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace robot_model_server_ros
{
    namespace
    {
        constexpr bool checkValidPubFreq(double val)
        {
            return val > 0.0 && val <= 1000.0;
        }

        template <typename MsgT>
        void copyToVector3(const Eigen::Vector3d &vec, MsgT &msg)
        {
            msg.x = vec.x();
            msg.y = vec.y();
            msg.z = vec.z();
        }

        template <typename MsgT>
        void copyToQuaternion(const Eigen::Quaterniond &quat, MsgT &msg)
        {
            msg.x = quat.x();
            msg.y = quat.y();
            msg.z = quat.z();
            msg.w = quat.w();
        }

        std::vector<geometry_msgs::msg::TransformStamped> toTransformStamped(
                const std::vector<robot_model_server::Transform> &transforms,
                const builtin_interfaces::msg::Time &stamp)
        {
            std::vector<geometry_msgs::msg::TransformStamped> result;
            result.reserve(transforms.size());
            for (const auto &tf : transforms)
            {
                geometry_msgs::msg::TransformStamped msg;
                msg.header.stamp = stamp;
                msg.header.frame_id = tf.frame_id;
                msg.child_frame_id = tf.child_frame_id;
                const Eigen::Vector3d &trans = tf.transform.translation();
                copyToVector3(trans, msg.transform.translation);
                const Eigen::Quaterniond quat(tf.transform.linear());
                copyToQuaternion(quat, msg.transform.rotation);
                result.push_back(msg);
            }
            return result;
        }

        visualization_msgs::msg::Marker toMarker(
                const robot_model_server::InertialDecomposition &dec,
                const std::string &ns,
                const int id,
                const std_msgs::msg::ColorRGBA &normal_color,
                const std_msgs::msg::ColorRGBA &scaled_color,
                const std_msgs::msg::ColorRGBA &tiny_color,
                const float alpha)
        {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = dec.link_name;
            marker.ns = ns;
            marker.id = id;
            marker.type = visualization_msgs::msg::Marker::CUBE;
            marker.action = visualization_msgs::msg::Marker::ADD;

            copyToVector3(dec.center_of_mass, marker.pose.position);

            const Eigen::Quaterniond quat(dec.eigenvectors);
            copyToQuaternion(quat, marker.pose.orientation);

            constexpr double MIN_DIM = 0.03;
            const bool all_tiny =
                    dec.box_size.x() < MIN_DIM && dec.box_size.y() < MIN_DIM && dec.box_size.z() < MIN_DIM;
            const double max_dim = dec.box_size.maxCoeff();

            if (all_tiny)
            {
                marker.scale.x = std::max(dec.box_size.x(), MIN_DIM);
                marker.scale.y = std::max(dec.box_size.y(), MIN_DIM);
                marker.scale.z = std::max(dec.box_size.z(), MIN_DIM);
                marker.color = tiny_color;
            }
            else if (max_dim > 1.0)
            {
                marker.scale.x = dec.box_size.x() / max_dim;
                marker.scale.y = dec.box_size.y() / max_dim;
                marker.scale.z = dec.box_size.z() / max_dim;
                marker.color = scaled_color;
            }
            else
            {
                marker.scale.x = dec.box_size.x();
                marker.scale.y = dec.box_size.y();
                marker.scale.z = dec.box_size.z();
                marker.color = normal_color;
            }
            marker.color.a = alpha;

            return marker;
        }

        visualization_msgs::msg::MarkerArray toMarkerArray(
                const std::vector<robot_model_server::InertialDecomposition> &decompositions,
                const float alpha)
        {
            std_msgs::msg::ColorRGBA normal_color;
            normal_color.r = 0.2;
            normal_color.g = 0.8;
            normal_color.b = 0.2;
            std_msgs::msg::ColorRGBA scaled_color;
            scaled_color.r = 0.8;
            scaled_color.g = 0.2;
            scaled_color.b = 0.2;
            std_msgs::msg::ColorRGBA tiny_color;
            tiny_color.r = 0.8;
            tiny_color.g = 0.6;
            tiny_color.b = 0.0;

            visualization_msgs::msg::MarkerArray result;
            result.markers.reserve(decompositions.size());
            for (size_t i = 0; i < decompositions.size(); ++i)
            {
                result.markers.push_back(toMarker(
                        decompositions.at(i),
                        decompositions.at(i).link_name,
                        static_cast<int>(i),
                        normal_color,
                        scaled_color,
                        tiny_color,
                        alpha));
            }
            return result;
        }

        visualization_msgs::msg::Marker toCumulativeMarker(
                const robot_model_server::CumulativeInertial &cumulative,
                const int id,
                const float alpha)
        {
            std_msgs::msg::ColorRGBA color;
            color.r = 0.2;
            color.g = 0.2;
            color.b = 0.8;
            return toMarker(cumulative, "cumulative_inertial", id, color, color, color, alpha);
        }
    }  // namespace

    Publisher::Publisher(const rclcpp::NodeOptions &options) : rclcpp::Node("robot_model_server", options)
    {
        rcl_interfaces::msg::ParameterDescriptor exclusive_desc;
        exclusive_desc.read_only = true;

        this->declare_parameter("model.description", std::string(""), exclusive_desc);
        this->declare_parameter("model.description_file", std::string(""), exclusive_desc);

        const std::string urdf_description = this->get_parameter("model.description").as_string();
        const std::string urdf_file = this->get_parameter("model.description_file").as_string();

        if (!urdf_description.empty() && !urdf_file.empty())
        {
            throw std::runtime_error("model.description and model.description_file are mutually exclusive");
        }

        std::string urdf_xml;
        if (!urdf_file.empty())
        {
            std::ifstream ifs(urdf_file);
            if (!ifs.is_open())
            {
                throw std::runtime_error("Failed to open model.description_file: " + urdf_file);
            }
            urdf_xml.assign(std::istreambuf_iterator<char>(ifs), std::istreambuf_iterator<char>());
        }
        else
        {
            urdf_xml = urdf_description;
        }

        if (urdf_xml.empty())
        {
            throw std::runtime_error("Either model.description or model.description_file must be provided");
        }

        description_pub_ =
                this->create_publisher<std_msgs::msg::String>("robot_description", rclcpp::QoS(1).transient_local());

        parameters_.init_inertial = this->declare_parameter("visualization.inertia", false);
        parameters_.init_cumulative_inertial = this->declare_parameter("inertia.cumulative", false);
        parameters_.inertia_tolerance = this->declare_parameter("inertia.tolerance", 0.0);
        visualization_alpha_ = this->declare_parameter("visualization.alpha", 0.5);

        if (parameters_.init_inertial)
        {
            inertial_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
                    "robot_model_server/inertia_visual", rclcpp::QoS(1).transient_local());
        }

        if (parameters_.init_cumulative_inertial)
        {
            cumulative_inertia_pub_ = this->create_publisher<geometry_msgs::msg::Inertia>(
                    "robot_model_server/cumulative_inertia", rclcpp::QoS(1).transient_local());
        }

        parameters_.frame_prefix = this->declare_parameter("transform.frame_prefix", std::string(""));

        core_.initialize(urdf_xml, parameters_);

        auto description_msg = std::make_unique<std_msgs::msg::String>();
        description_msg->data = urdf_xml;
        description_pub_->publish(std::move(description_msg));  // NOLINT

        publish_frequency_ = this->declare_parameter("transform.frequency", 20.0);
        if (!checkValidPubFreq(publish_frequency_))
        {
            throw std::runtime_error("transform.frequency must be between 0 (exclusive) and 1000");
        }

        ignore_timestamp_ = this->declare_parameter("joint_states.ignore_timestamp", false);

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
        if (parameters_.init_inertial)
        {
            publishInertialDecompositions();
        }
        if (parameters_.init_cumulative_inertial)
        {
            publishCumulativeInertial();
        }
    }

    void Publisher::publishFixedTransforms()
    {
        const auto &tf_transforms = core_.getFixedTransforms();
        static_tf_broadcaster_->sendTransform(toTransformStamped(tf_transforms, this->now()));
    }

    void Publisher::publishInertialDecompositions()
    {
        const auto &decompositions = core_.getInertialDecompositions();
        auto marker_array = std::make_unique<visualization_msgs::msg::MarkerArray>(
                toMarkerArray(decompositions, static_cast<float>(visualization_alpha_)));
        if (parameters_.init_cumulative_inertial)
        {
            marker_array->markers.push_back(toCumulativeMarker(
                    core_.getCumulativeInertial(),
                    static_cast<int>(marker_array->markers.size()),
                    static_cast<float>(visualization_alpha_)));
        }
        inertial_marker_pub_->publish(std::move(marker_array));
    }

    void Publisher::publishCumulativeInertial()
    {
        const auto &cumulative = core_.getCumulativeInertial();
        const std::string root_frame = cumulative.link_name;

        geometry_msgs::msg::TransformStamped com_tf;
        com_tf.header.stamp = this->now();
        com_tf.header.frame_id = root_frame;
        com_tf.child_frame_id = parameters_.frame_prefix + "cumulative_center_of_mass";
        copyToVector3(cumulative.center_of_mass, com_tf.transform.translation);
        com_tf.transform.rotation.x = 0.0;
        com_tf.transform.rotation.y = 0.0;
        com_tf.transform.rotation.z = 0.0;
        com_tf.transform.rotation.w = 1.0;
        static_tf_broadcaster_->sendTransform(com_tf);

        auto inertia_msg = std::make_unique<geometry_msgs::msg::Inertia>();
        inertia_msg->m = cumulative.mass;
        copyToVector3(cumulative.center_of_mass, inertia_msg->com);
        inertia_msg->ixx = cumulative.inertia(0, 0);
        inertia_msg->ixy = cumulative.inertia(0, 1);
        inertia_msg->ixz = cumulative.inertia(0, 2);
        inertia_msg->iyy = cumulative.inertia(1, 1);
        inertia_msg->iyz = cumulative.inertia(1, 2);
        inertia_msg->izz = cumulative.inertia(2, 2);
        cumulative_inertia_pub_->publish(std::move(inertia_msg));
    }

    void Publisher::callbackJointState(const sensor_msgs::msg::JointState::ConstSharedPtr &state)
    {
        if (state->name.size() != state->position.size())
        {
            RCLCPP_ERROR(
                    get_logger(),
                    "Robot state server ignored a JointState message with mismatched joint names and positions.");
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
            const auto tf_transforms = core_.getTransforms(state->name, state->position);
            tf_broadcaster_->sendTransform(toTransformStamped(tf_transforms, state->header.stamp));

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
        rclcpp::spin(std::make_shared<robot_model_server_ros::Publisher>(rclcpp::NodeOptions()));
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
