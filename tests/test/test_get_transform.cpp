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

#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

#include "gtest/gtest.h"

#include "robot_model_server/robot_model_server.hpp"

static constexpr double EPS = 1e-6;

namespace
{
const std::string &movingJointUrdf()
{
    static const std::string urdf = R"(
<robot name="test_robot">
  <link name="link1" />
  <link name="link2" />
  <joint name="joint1" type="continuous">
    <parent link="link1"/>
    <child link="link2"/>
    <origin xyz="5 0 0" rpy="0 0 1.57" />
  </joint>
</robot>
)";
    return urdf;
}

const std::string &fixedJointUrdf()
{
    static const std::string urdf = R"(
<robot name="test_robot">
  <link name="link1" />
  <link name="link2" />
  <joint name="joint1" type="fixed">
    <parent link="link1"/>
    <child link="link2"/>
    <origin xyz="5 0 0" rpy="0 0 1.57" />
  </joint>
</robot>
)";
    return urdf;
}

void initModel(robot_model_server::Model &model, const std::string &urdf, const std::string &prefix = "")
{
    robot_model_server::Model::Parameters params;
    params.frame_prefix = prefix;
    model.initialize(urdf, params);
}
} // namespace

TEST(TestGetTransform, SameFrameReturnsIdentity)
{
    robot_model_server::Model model;
    initModel(model, movingJointUrdf());
    const auto tf = model.getTransform("link1", "link1");

    EXPECT_EQ(tf.frame_id, "link1");
    EXPECT_EQ(tf.child_frame_id, "link1");
    EXPECT_TRUE(tf.transform.isApprox(Eigen::Isometry3d::Identity(), EPS));
}

TEST(TestGetTransform, FixedJoint)
{
    robot_model_server::Model model;
    initModel(model, fixedJointUrdf());
    const auto tf = model.getTransform("link1", "link2");

    EXPECT_EQ(tf.frame_id, "link1");
    EXPECT_EQ(tf.child_frame_id, "link2");
    EXPECT_NEAR(tf.transform.translation().x(), 5.0, EPS);
    EXPECT_NEAR(tf.transform.translation().y(), 0.0, EPS);
    EXPECT_NEAR(tf.transform.translation().z(), 0.0, EPS);
}

TEST(TestGetTransform, MovingJointAtZero)
{
    robot_model_server::Model model;
    initModel(model, movingJointUrdf());
    const auto tf = model.getTransform("link1", "link2", {"joint1"}, {0.0});

    EXPECT_NEAR(tf.transform.translation().x(), 5.0, EPS);
    EXPECT_NEAR(tf.transform.translation().y(), 0.0, EPS);
    EXPECT_NEAR(tf.transform.translation().z(), 0.0, EPS);
}

TEST(TestGetTransform, MovingJointAtPiHalf)
{
    robot_model_server::Model model;
    initModel(model, movingJointUrdf());
    const std::vector<std::string> names = {"joint1"};
    const std::vector<double> positions = {M_PI / 2.0};

    const auto tf = model.getTransform("link1", "link2", names, positions);
    const auto per_joint = model.getTransforms(names, positions);
    ASSERT_EQ(per_joint.size(), 1u);

    EXPECT_NEAR(tf.transform.translation().x(), 5.0, EPS);
    EXPECT_NEAR(tf.transform.translation().y(), 0.0, EPS);
    EXPECT_NEAR(tf.transform.translation().z(), 0.0, EPS);

    EXPECT_TRUE(tf.transform.linear().isApprox(per_joint.at(0).transform.linear(), EPS));
}

TEST(TestGetTransform, ThrowsWhenJointNotProvided)
{
    robot_model_server::Model model;
    initModel(model, movingJointUrdf());
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-result"
    EXPECT_THROW(model.getTransform("link1", "link2"), std::invalid_argument);
#pragma GCC diagnostic pop
}

TEST(TestGetTransform, ThrowsWhenNamesSizesMismatch)
{
    robot_model_server::Model model;
    initModel(model, movingJointUrdf());
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-result"
    EXPECT_THROW(model.getTransform("link1", "link2", {"joint1"}, {}), std::invalid_argument);
#pragma GCC diagnostic pop
}

TEST(TestGetTransform, ThrowsForNonexistentFrames)
{
    robot_model_server::Model model;
    initModel(model, movingJointUrdf());
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-result"
    EXPECT_THROW(model.getTransform("link1", "nonexistent"), std::invalid_argument);
#pragma GCC diagnostic pop
}

TEST(TestGetTransform, WithFramePrefix)
{
    robot_model_server::Model model;
    initModel(model, movingJointUrdf(), "prefix/");
    const auto tf = model.getTransform("prefix/link1", "prefix/link2", {"joint1"}, {0.0});

    EXPECT_EQ(tf.frame_id, "prefix/link1");
    EXPECT_EQ(tf.child_frame_id, "prefix/link2");
    EXPECT_NEAR(tf.transform.translation().x(), 5.0, EPS);
}

TEST(TestGetTransform, WithFramePrefixUnprefixedInput)
{
    robot_model_server::Model model;
    initModel(model, movingJointUrdf(), "prefix/");
    const auto tf = model.getTransform("link1", "link2", {"joint1"}, {0.0});

    EXPECT_EQ(tf.frame_id, "link1");
    EXPECT_EQ(tf.child_frame_id, "link2");
    EXPECT_NEAR(tf.transform.translation().x(), 5.0, EPS);
}

TEST(TestGetTransform, WithFramePrefixMixedInput)
{
    robot_model_server::Model model;
    initModel(model, movingJointUrdf(), "prefix/");
    const auto tf = model.getTransform("prefix/link1", "link2", {"joint1"}, {0.0});

    EXPECT_EQ(tf.frame_id, "prefix/link1");
    EXPECT_EQ(tf.child_frame_id, "link2");
    EXPECT_NEAR(tf.transform.translation().x(), 5.0, EPS);
}

TEST(TestGetTransform, InverseTransformConsistency)
{
    robot_model_server::Model model;
    initModel(model, movingJointUrdf());
    const std::vector<std::string> names = {"joint1"};
    const std::vector<double> positions = {M_PI / 3.0};

    const auto tf_fwd = model.getTransform("link1", "link2", names, positions);
    const auto tf_rev = model.getTransform("link2", "link1", names, positions);

    const Eigen::Isometry3d product = tf_fwd.transform * tf_rev.transform;
    EXPECT_TRUE(product.isApprox(Eigen::Isometry3d::Identity(), 1e-6));
}

TEST(TestGetTransform, CrossVerifyWithGetTransforms)
{
    robot_model_server::Model model;
    initModel(model, movingJointUrdf());
    const std::vector<std::string> names = {"joint1"};
    const std::vector<double> positions = {M_PI / 4.0};

    const auto per_joint = model.getTransforms(names, positions);
    ASSERT_EQ(per_joint.size(), 1u);

    const auto cumulative = model.getTransform("link1", "link2", names, positions);
    EXPECT_TRUE(cumulative.transform.isApprox(per_joint.at(0).transform, EPS));
}
