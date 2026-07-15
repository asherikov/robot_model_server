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

#pragma once

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>

namespace robot_model_server
{
    struct Transform
    {
        Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
        std::string frame_id;
        std::string child_frame_id;
    };

    struct InertialDecomposition
    {
        std::string link_name;
        Eigen::Vector3d center_of_mass = Eigen::Vector3d::Zero();
        double mass = 0.0;
        Eigen::Vector3d eigenvalues = Eigen::Vector3d::Zero();
        Eigen::Matrix3d eigenvectors = Eigen::Matrix3d::Identity();
        Eigen::Vector3d box_size = Eigen::Vector3d::Zero();
    };

    struct CumulativeInertial : InertialDecomposition
    {
        Eigen::Matrix3d inertia = Eigen::Matrix3d::Zero();
    };

    class Model
    {
    public:
        struct Parameters
        {
            std::string frame_prefix;
            double inertia_tolerance = 0.0;
            bool init_inertial = false;
            bool init_cumulative_inertial = false;
        };

        Model();
        ~Model();

        Model(Model &&) = default;
        Model &operator=(Model &&) = default;

        void initialize(const std::string &urdf_xml, const Parameters &parameters);

        [[nodiscard]] std::vector<Transform> getTransforms(
                const std::vector<std::string> &joint_names,
                const std::vector<double> &joint_positions) const;

        [[nodiscard]] const std::vector<Transform> &getFixedTransforms() const;

        [[nodiscard]] const std::vector<InertialDecomposition> &getInertialDecompositions() const;

        [[nodiscard]] const CumulativeInertial &getCumulativeInertial() const;

    private:
        class Implementation;
        std::unique_ptr<Implementation> pimpl_;
    };
}  // namespace robot_model_server
