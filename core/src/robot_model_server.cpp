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

#include "robot_model_server/robot_model_server.hpp"

#include <algorithm>
#include <map>
#include <memory>
#include <numeric>
#include <stdexcept>
#include <string>
#include <vector>

#include "kdl/tree.hpp"
#include "kdl_parser/kdl_parser.hpp"

// cppcheck-suppress preprocessorErrorDirective
#if __has_include(<urdf/model.hpp>)
#    include "urdf/model.hpp"
#else
#    include "urdf/model.h"
#endif

namespace robot_model_server
{
    namespace
    {
        inline void kdlToIsometry(const KDL::Frame &k, Eigen::Isometry3d &out)
        {
            out.translation() << k.p.x(), k.p.y(), k.p.z();
            out.linear() = Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(k.M.data);
        }

        struct LinkInertial
        {
            Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
            Eigen::Vector3d com = Eigen::Vector3d::Zero();
            double mass = 0.0;
            Eigen::Matrix3d inertia = Eigen::Matrix3d::Zero();
        };

        bool isNegligible(const urdf::Inertial &inertial, double tolerance)
        {
            return std::abs(inertial.mass) < tolerance && std::abs(inertial.ixx) < tolerance
                   && std::abs(inertial.ixy) < tolerance && std::abs(inertial.ixz) < tolerance
                   && std::abs(inertial.iyy) < tolerance && std::abs(inertial.iyz) < tolerance
                   && std::abs(inertial.izz) < tolerance;
        }

        void ensureProperRotation(Eigen::Matrix3d &eigenvectors)
        {
            if (eigenvectors.determinant() < 0)
            {
                eigenvectors.col(2) *= -1;
            }
        }

        // Compute the dimensions of a uniform solid box with equivalent mass and
        // principal moments of inertia.  A box with side lengths (a, b, c) and
        // mass m has principal moments:
        //
        //   I1 = m/12 * (b^2 + c^2)
        //   I2 = m/12 * (a^2 + c^2)
        //   I3 = m/12 * (a^2 + b^2)
        //
        // Inverting these equations and solving for a, b, c yields:
        //
        //   a = sqrt(6 * (I2 + I3 - I1) / m)
        //   b = sqrt(6 * (I3 + I1 - I2) / m)
        //   c = sqrt(6 * (I1 + I2 - I3) / m)
        //
        // The triangle inequality on principal moments (I1 + I2 >= I3, etc.)
        // guarantees the radicands are non-negative.  A negative value due to
        // numerical error is clamped to zero.
        //
        // Reference: gz-math MassMatrix3::EquivalentBox
        // https://github.com/gazebosim/gz-math/blob/main/include/gz/math/MassMatrix3.hh
        Eigen::Vector3d equivalentBoxSize(const Eigen::Vector3d &eigenvalues, double mass)
        {
            const double i1 = eigenvalues.x();
            const double i2 = eigenvalues.y();
            const double i3 = eigenvalues.z();
            return {
                std::sqrt(std::max(0.0, 6.0 * (i2 + i3 - i1) / mass)),
                std::sqrt(std::max(0.0, 6.0 * (i3 + i1 - i2) / mass)),
                std::sqrt(std::max(0.0, 6.0 * (i1 + i2 - i3) / mass)),
            };
        }

        template <typename T>
        void sortByKeys(std::vector<std::string> &keys, std::vector<T> &values)
        {
            std::vector<size_t> indices(keys.size());
            std::iota(indices.begin(), indices.end(), 0);
            std::sort(indices.begin(), indices.end(), [&keys](size_t a, size_t b) { return keys.at(a) < keys.at(b); });

            std::vector<std::string> sorted_keys;
            sorted_keys.reserve(keys.size());
            std::vector<T> sorted_values;
            sorted_values.reserve(values.size());
            for (const size_t idx : indices)
            {
                sorted_keys.push_back(std::move(keys.at(idx)));
                sorted_values.push_back(std::move(values.at(idx)));
            }
            keys = std::move(sorted_keys);
            values = std::move(sorted_values);
        }

    }  // namespace

    class Model::Implementation
    {
    public:
        class MovableJoint
        {
        public:
            explicit MovableJoint(
                    const KDL::Segment &p_segment,
                    std::string p_joint_name,
                    std::string p_parent_frame_id,
                    std::string p_child_frame_id,
                    const std::string &frame_prefix)
              : segment(p_segment)
              , joint_name(std::move(p_joint_name))
              , parent_frame_id(frame_prefix + std::move(p_parent_frame_id))
              , child_frame_id(frame_prefix + std::move(p_child_frame_id))
            {
            }

            KDL::Segment segment;
            std::string joint_name;
            std::string parent_frame_id;
            std::string child_frame_id;
        };

        struct MimicJoint
        {
            std::string joint_name;
            urdf::JointMimic mimic;
        };

        std::vector<MovableJoint> movable_joints_;
        std::vector<Transform> fixed_transforms_;
        std::vector<MimicJoint> mimic_joints_;
        std::vector<InertialDecomposition> inertial_decompositions_;
        urdf::Model model_;
        KDL::Tree tree_;
        CumulativeInertial cumulative_inertial_;
        std::string frame_prefix_;

        static void parseURDF(const std::string &urdf_xml, urdf::Model &model, KDL::Tree &tree)
        {
            if (!model.initString(urdf_xml))
            {
                throw std::runtime_error("Unable to initialize urdf::model from robot description");
            }

            if (!kdl_parser::treeFromUrdfModel(model, tree))
            {
                throw std::runtime_error("Failed to extract kdl tree from robot description");
            }
        }

        void initialize(const std::string &urdf_xml, const Parameters &parameters)
        {
            if (urdf_xml.empty())
            {
                throw std::runtime_error("URDF description must not be empty");
            }

            parseURDF(urdf_xml, model_, tree_);

            mimic_joints_.clear();
            for (const auto &[joint_name, joint] : model_.joints_)
            {
                if (joint->mimic)
                {
                    mimic_joints_.push_back(MimicJoint{joint_name, *joint->mimic});
                }
            }

            movable_joints_.clear();
            fixed_transforms_.clear();
            addChildren(model_, tree_.getRootSegment(), parameters.frame_prefix);
            frame_prefix_ = parameters.frame_prefix;

            std::sort(movable_joints_.begin(), movable_joints_.end(), [](const MovableJoint &a, const MovableJoint &b) {
                return a.joint_name < b.joint_name;
            });

            inertial_decompositions_.clear();
            if (parameters.init_inertial)
            {
                computeInertialDecompositions(parameters.inertia_tolerance);
            }

            cumulative_inertial_ = {};
            cumulative_inertial_.link_name = frame_prefix_ + GetTreeElementSegment(tree_.getRootSegment()->second).getName();
            if (parameters.init_cumulative_inertial)
            {
                computeCumulativeInertial(parameters.inertia_tolerance);
            }
        }

        void computeInertialDecompositions(double tolerance)
        {
            inertial_decompositions_.reserve(model_.links_.size());
            for (const auto &[name, link] : model_.links_)
            {
                if (!link || !link->inertial)
                {
                    continue;
                }

                const auto &inertial = *link->inertial;
                if (isNegligible(inertial, tolerance))
                {
                    continue;
                }

                InertialDecomposition result;
                result.link_name = frame_prefix_ + name;
                result.center_of_mass = Eigen::Vector3d(
                        inertial.origin.position.x, inertial.origin.position.y, inertial.origin.position.z);
                result.mass = inertial.mass;

                Eigen::Matrix3d inertia_matrix;
                inertia_matrix << inertial.ixx, inertial.ixy, inertial.ixz,
                                  inertial.ixy, inertial.iyy, inertial.iyz,
                                  inertial.ixz, inertial.iyz, inertial.izz;

                const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(inertia_matrix);
                result.eigenvalues = solver.eigenvalues();
                result.eigenvectors = solver.eigenvectors();
                ensureProperRotation(result.eigenvectors);
                result.box_size = equivalentBoxSize(result.eigenvalues, result.mass);

                inertial_decompositions_.push_back(std::move(result));
            }
        }

        void addChildren(
                const urdf::Model &model,
                const KDL::SegmentMap::const_iterator segment,
                const std::string &frame_prefix)
        {
            const std::string &root = GetTreeElementSegment(segment->second).getName();

            const std::vector<KDL::SegmentMap::const_iterator> children = GetTreeElementChildren(segment->second);
            for (const KDL::SegmentMap::const_iterator &child_it : children)
            {
                const KDL::Segment &child = GetTreeElementSegment(child_it->second);
                if (child.getJoint().getType() == KDL::Joint::None)
                {
                    const auto joint = model.getJoint(child.getJoint().getName());
                    if (!joint || joint->type != urdf::Joint::FLOATING)
                    {
                        Transform tf;
                        kdlToIsometry(child.pose(0), tf.transform);
                        tf.frame_id = frame_prefix + root;
                        tf.child_frame_id = frame_prefix + child.getName();
                        fixed_transforms_.push_back(std::move(tf));
                    }
                }
                else
                {
                    movable_joints_.emplace_back(child, child.getJoint().getName(), root, child.getName(), frame_prefix);
                }
                addChildren(model, child_it, frame_prefix);
            }
        }

        [[nodiscard]] std::vector<Transform> getTransforms(
                const std::vector<std::string> &joint_names,
                const std::vector<double> &joint_positions) const
        {
            if (joint_names.size() != joint_positions.size())
            {
                throw std::invalid_argument("joint_names and joint_positions must have the same size");
            }

            std::vector<std::string> names = joint_names;
            std::vector<double> positions = joint_positions;
            sortByKeys(names, positions);

            for (const auto &mimic_joint : mimic_joints_)
            {
                const auto &mimic = mimic_joint.mimic;
                auto it = std::lower_bound(names.begin(), names.end(), mimic.joint_name);
                if (it != names.end() && *it == mimic.joint_name)
                {
                    const size_t src_idx = static_cast<size_t>(std::distance(names.begin(), it));
                    const double pos = (positions.at(src_idx) * mimic.multiplier) + mimic.offset;
                    names.push_back(mimic_joint.joint_name);
                    positions.push_back(pos);
                }
            }

            std::vector<Transform> tf_transforms;
            for (size_t i = 0; i < names.size() && i < positions.size(); ++i)
            {
                auto it = std::lower_bound(
                        movable_joints_.begin(), movable_joints_.end(), names.at(i),
                        [](const MovableJoint &joint, const std::string &name) { return joint.joint_name < name; });
                if (it != movable_joints_.end() && it->joint_name == names.at(i))
                {
                    Transform tf;
                    kdlToIsometry(it->segment.pose(positions.at(i)), tf.transform);
                    tf.frame_id = it->parent_frame_id;
                    tf.child_frame_id = it->child_frame_id;
                    tf_transforms.push_back(std::move(tf));
                }
            }
            return tf_transforms;
        }

        void collectLinkInertials(
                const KDL::SegmentMap::const_iterator &segment_it,
                const Eigen::Isometry3d &parent_transform,
                std::vector<LinkInertial> &collected,
                double tolerance) const
        {
            const KDL::Segment &segment = GetTreeElementSegment(segment_it->second);
            Eigen::Isometry3d segment_transform;
            kdlToIsometry(segment.pose(0), segment_transform);
            const Eigen::Isometry3d link_transform = parent_transform * segment_transform;

            const auto link_it = model_.links_.find(segment.getName());
            if (link_it != model_.links_.end() && link_it->second && link_it->second->inertial)
            {
                const auto &inertial = *link_it->second->inertial;
                if (!isNegligible(inertial, tolerance))
                {
                    LinkInertial entry;
                    entry.transform = link_transform;
                    entry.com =
                            link_transform
                            * Eigen::Vector3d(
                                    inertial.origin.position.x, inertial.origin.position.y, inertial.origin.position.z);
                    entry.mass = inertial.mass;
                    entry.inertia << inertial.ixx, inertial.ixy, inertial.ixz, inertial.ixy, inertial.iyy, inertial.iyz,
                            inertial.ixz, inertial.iyz, inertial.izz;
                    collected.push_back(std::move(entry));
                }
            }

            const std::vector<KDL::SegmentMap::const_iterator> children = GetTreeElementChildren(segment_it->second);
            for (const auto &child_it : children)
            {
                collectLinkInertials(child_it, link_transform, collected, tolerance);
            }
        }

        void computeCumulativeInertial(double tolerance)
        {
            std::vector<LinkInertial> collected;
            collectLinkInertials(tree_.getRootSegment(), Eigen::Isometry3d::Identity(), collected, tolerance);

            cumulative_inertial_.mass = 0.0;
            cumulative_inertial_.center_of_mass = Eigen::Vector3d::Zero();
            cumulative_inertial_.inertia = Eigen::Matrix3d::Zero();
            Eigen::Vector3d weighted_com = Eigen::Vector3d::Zero();
            for (const auto &entry : collected)
            {
                cumulative_inertial_.mass += entry.mass;
                weighted_com += entry.mass * entry.com;
            }

            if (cumulative_inertial_.mass <= tolerance)
            {
                return;
            }

            cumulative_inertial_.center_of_mass = weighted_com / cumulative_inertial_.mass;

            for (const auto &entry : collected)
            {
                const Eigen::Matrix3d R = entry.transform.linear();
                const Eigen::Vector3d d = entry.com - cumulative_inertial_.center_of_mass;
                const Eigen::Matrix3d I_rotated = R * entry.inertia * R.transpose();
                const Eigen::Matrix3d I_parallel =
                        entry.mass * (d.squaredNorm() * Eigen::Matrix3d::Identity() - d * d.transpose());
                cumulative_inertial_.inertia += I_rotated + I_parallel;
            }

            const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(cumulative_inertial_.inertia);
            cumulative_inertial_.eigenvalues = solver.eigenvalues();
            cumulative_inertial_.eigenvectors = solver.eigenvectors();
            ensureProperRotation(cumulative_inertial_.eigenvectors);
            cumulative_inertial_.box_size =
                    equivalentBoxSize(cumulative_inertial_.eigenvalues, cumulative_inertial_.mass);
        }
    };

    Model::Model() : pimpl_(std::make_unique<Implementation>())
    {
    }

    Model::~Model() = default;

    void Model::initialize(const std::string &urdf_xml, const Parameters &parameters)
    {
        pimpl_->initialize(urdf_xml, parameters);
    }

    [[nodiscard]] std::vector<Transform> Model::getTransforms(
            const std::vector<std::string> &joint_names,
            const std::vector<double> &joint_positions) const
    {
        return pimpl_->getTransforms(joint_names, joint_positions);
    }

    [[nodiscard]] const std::vector<Transform> &Model::getFixedTransforms() const
    {
        return pimpl_->fixed_transforms_;
    }

    [[nodiscard]] const std::vector<InertialDecomposition> &Model::getInertialDecompositions() const
    {
        return pimpl_->inertial_decompositions_;
    }

    [[nodiscard]] const CumulativeInertial &Model::getCumulativeInertial() const
    {
        return pimpl_->cumulative_inertial_;
    }
}  // namespace robot_model_server
