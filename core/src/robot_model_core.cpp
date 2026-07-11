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
// INTERRUPTION HOWEVER CAUSED AND HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
// WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
// OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
// ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include "robot_model_server_core/robot_model_core.hpp"

#include <chrono>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "kdl/tree.hpp"
#include "kdl_parser/kdl_parser.hpp"

// cppcheck-suppress preprocessorErrorDirective
#if __has_include(<urdf/model.hpp>)
#    include "urdf/model.hpp"
#else
#    include "urdf/model.h"
#endif

namespace robot_model_server_core
{

    namespace
    {

        inline Transform kdlToTransform(const KDL::Frame &k)
        {
            Transform t;
            t.translation.x = k.p.x();
            t.translation.y = k.p.y();
            t.translation.z = k.p.z();
            k.M.GetQuaternion(t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w);
            return t;
        }

    }  // namespace

    class RobotModelCore::Implementation
    {
    public:
        class SegmentPair final
        {
        public:
            explicit SegmentPair(const KDL::Segment &p_segment, std::string p_root, std::string p_tip)
              : segment(p_segment), root(std::move(p_root)), tip(std::move(p_tip))
            {
            }

            KDL::Segment segment;
            std::string root;
            std::string tip;
        };

        using MimicMap = std::map<std::string, urdf::JointMimicSharedPtr>;

        std::map<std::string, SegmentPair> segments_;
        std::map<std::string, SegmentPair> segments_fixed_;
        MimicMap mimic_;

        static KDL::Tree parseURDF(const std::string &urdf_xml, urdf::Model &model)
        {
            if (!model.initString(urdf_xml))
            {
                throw std::runtime_error("Unable to initialize urdf::model from robot description");
            }

            KDL::Tree tree;
            if (!kdl_parser::treeFromUrdfModel(model, tree))
            {
                throw std::runtime_error("Failed to extract kdl tree from robot description");
            }

            return tree;
        }

        void setupURDF(const std::string &urdf_xml)
        {
            if (urdf_xml.empty())
            {
                throw std::runtime_error("robot_description parameter must not be empty");
            }

            urdf::Model model;
            const KDL::Tree tree = parseURDF(urdf_xml, model);

            mimic_.clear();
            for (const auto &[joint_name, joint] : model.joints_)
            {
                if (joint->mimic)
                {
                    auto jm = std::make_shared<urdf::JointMimic>();
                    jm->offset = joint->mimic->offset;
                    jm->multiplier = joint->mimic->multiplier;
                    jm->joint_name = joint->mimic->joint_name;
                    mimic_[joint_name] = jm;
                }
            }

            segments_.clear();
            segments_fixed_.clear();
            addChildren(model, tree.getRootSegment());
        }

        void addChildren(const urdf::Model &model, const KDL::SegmentMap::const_iterator segment)
        {
            const std::string &root = GetTreeElementSegment(segment->second).getName();

            const std::vector<KDL::SegmentMap::const_iterator> children = GetTreeElementChildren(segment->second);
            for (const KDL::SegmentMap::const_iterator &child_it : children)
            {
                const KDL::Segment &child = GetTreeElementSegment(child_it->second);
                const SegmentPair s(GetTreeElementSegment(child_it->second), root, child.getName());
                if (child.getJoint().getType() == KDL::Joint::None)
                {
                    if (model.getJoint(child.getJoint().getName())
                        && model.getJoint(child.getJoint().getName())->type == urdf::Joint::FLOATING)
                    {
                    }
                    else
                    {
                        segments_fixed_.emplace(child.getJoint().getName(), s);
                    }
                }
                else
                {
                    segments_.emplace(child.getJoint().getName(), s);
                }
                addChildren(model, child_it);
            }
        }

        void computeMimicJoints(std::map<std::string, double> &joint_positions) const
        {
            for (const auto &[mimic_name, mimic] : mimic_)
            {
                if (auto it = joint_positions.find(mimic->joint_name); it != joint_positions.end())
                {
                    const double pos = (it->second * mimic->multiplier) + mimic->offset;
                    joint_positions.emplace(mimic_name, pos);
                }
            }
        }

        [[nodiscard]] std::vector<Transform> getTransforms(
                const std::map<std::string, double> &joint_positions,
                const Time &time,
                const std::string &frame_prefix) const
        {
            std::vector<Transform> tf_transforms;
            tf_transforms.reserve(joint_positions.size());

            for (const auto &[joint_name, position] : joint_positions)
            {
                auto seg = segments_.find(joint_name);
                if (seg != segments_.end())
                {
                    Transform tf_transform = kdlToTransform(seg->second.segment.pose(position));
                    tf_transform.stamp = time;
                    tf_transform.frame_id = frame_prefix + seg->second.root;
                    tf_transform.child_frame_id = frame_prefix + seg->second.tip;
                    tf_transforms.push_back(tf_transform);
                }
            }
            return tf_transforms;
        }

        [[nodiscard]] std::vector<Transform> getFixedTransforms(const Time &time, const std::string &frame_prefix) const
        {
            std::vector<Transform> tf_transforms;
            tf_transforms.reserve(segments_fixed_.size());

            for (const auto &[joint_name, seg] : segments_fixed_)
            {
                Transform tf_transform = kdlToTransform(seg.segment.pose(0));
                tf_transform.stamp = time;
                tf_transform.frame_id = frame_prefix + seg.root;
                tf_transform.child_frame_id = frame_prefix + seg.tip;
                tf_transforms.push_back(tf_transform);
            }
            return tf_transforms;
        }
    };

    RobotModelCore::RobotModelCore() : pimpl_(std::make_unique<Implementation>())
    {
    }

    RobotModelCore::~RobotModelCore() = default;

    bool RobotModelCore::isValidURDF(const std::string &urdf_xml)
    {
        if (urdf_xml.empty())
        {
            return false;
        }

        try
        {
            urdf::Model dummy_model;
            Implementation::parseURDF(urdf_xml, dummy_model);
            return true;
        }
        catch (const std::runtime_error &)
        {
            return false;
        }
    }

    void RobotModelCore::setupURDF(const std::string &urdf_xml)
    {
        pimpl_->setupURDF(urdf_xml);
    }

    void RobotModelCore::computeMimicJoints(std::map<std::string, double> &joint_positions) const
    {
        pimpl_->computeMimicJoints(joint_positions);
    }

    [[nodiscard]] std::vector<Transform> RobotModelCore::getTransforms(
            const std::map<std::string, double> &joint_positions,
            const Time &time,
            const std::string &frame_prefix) const
    {
        return pimpl_->getTransforms(joint_positions, time, frame_prefix);
    }

    [[nodiscard]] std::vector<Transform> RobotModelCore::getFixedTransforms(const Time &time, const std::string &frame_prefix) const
    {
        return pimpl_->getFixedTransforms(time, frame_prefix);
    }

}  // namespace robot_model_server_core
