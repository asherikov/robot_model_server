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

#ifndef ROBOT_MODEL_SERVER_CORE_ROBOT_MODEL_CORE_HPP_
#define ROBOT_MODEL_SERVER_CORE_ROBOT_MODEL_CORE_HPP_

#include <chrono>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "kdl/tree.hpp"
#include "urdf/model.h"

namespace robot_model_server_core
{

    using MimicMap = std::map<std::string, urdf::JointMimicSharedPtr>;

    using Time = std::chrono::nanoseconds;

    struct Transform
    {
        struct Translation
        {
            double x = 0.0;
            double y = 0.0;
            double z = 0.0;
        };

        struct Rotation
        {
            double x = 0.0;
            double y = 0.0;
            double z = 0.0;
            double w = 1.0;
        };

        Translation translation;
        Rotation rotation;
        std::string frame_id;
        std::string child_frame_id;
        Time stamp{};
    };

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

    class RobotModelCore
    {
    public:
        RobotModelCore() = default;

        void setupURDF(const std::string &urdf_xml);

        static bool isValidURDF(const std::string &urdf_xml);

        static constexpr bool checkValidPubFreq(double val)
        {
            return val > 0.0 && val <= 1000.0;
        }

        void computeMimicJoints(std::map<std::string, double> &joint_positions) const;

        std::vector<Transform> getTransforms(
                const std::map<std::string, double> &joint_positions,
                const Time &time,
                const std::string &frame_prefix) const;

        std::vector<Transform> getFixedTransforms(const Time &time, const std::string &frame_prefix) const;

        const MimicMap &getMimicMap() const
        {
            return mimic_;
        }

        const std::map<std::string, SegmentPair> &getSegments() const
        {
            return segments_;
        }

        const std::map<std::string, SegmentPair> &getFixedSegments() const
        {
            return segments_fixed_;
        }

    private:
        static KDL::Tree parseURDF(const std::string &urdf_xml, urdf::Model &model);

        void addChildren(const urdf::Model &model, const KDL::SegmentMap::const_iterator segment);

        std::map<std::string, SegmentPair> segments_;
        std::map<std::string, SegmentPair> segments_fixed_;
        MimicMap mimic_;
    };

}  // namespace robot_model_server_core

#endif  // ROBOT_MODEL_SERVER_CORE_ROBOT_MODEL_CORE_HPP_
