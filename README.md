Robot Model Server
=================

This repository contains the Robot Model Server, a node and library to publish the state of a robot to tf2.
At startup time, Robot Model Server is supplied with a kinematic tree model (URDF) of the robot.
It then subscribes to the `joint_states` topic (of type `sensor_msgs/msg/JointState`) to get individual joint states.
These joint states are used to update the kinematic tree model, and the resulting 3D poses are then published to tf2.

Robot Model Server deals with two different "classes" of joint types: fixed and movable.
Fixed joints (with the type "fixed") are published to the transient_local `/tf_static` topic once on startup (transient_local topics keep a history of what they published, so a later subscription can always get the latest state of the world).
Movable joints are published to the regular `/tf` topic any time the appropriate joint is updated in the `joint_states` message.

By default, the robot description must be provided via the `robot_description` parameter.

Examples showing how to pass the `robot_description` parameter using a launch file are available in the `launch` subdirectory of `robot_model_server_tests`.

Repository Structure
--------------------

The repository is organized into three packages:

* **`robot_model_server_core`** — Core library with minimal ROS dependencies. Provides URDF parsing, KDL tree walking, transform computation, and mimic joint handling. Depends only on `builtin_interfaces`, `geometry_msgs`, `kdl_parser`, `urdf`, and `orocos_kdl`.

* **`robot_model_server_ros`** — ROS 2 wrapper around the core library. Contains the `robot_model_server_ros` node, ROS publishers/subscribers, tf2 broadcasters, and parameter handling.

* **`robot_model_server_tests`** — Launch-based integration tests for the ROS wrapper. Includes test URDFs, example launch files, and launch-based tests.

Published Topics
----------------
* `robot_description` (`std_msgs/msg/String`) - The description of the robot URDF as a string. Republishes the value set in the `robot_description` parameter, which is useful for informing other tools of the current robot model. Published using the "transient local" quality of service, so subscribers should also use "transient local".
* `tf` (`tf2_msgs/msg/TFMessage`) - The transforms corresponding to the movable joints of the robot.
* `tf_static` (`tf2_msgs/msg/TFMessage`) - The transforms corresponding to the static joints of the robot.

Subscribed Topics
-----------------
* `joint_states` (`sensor_msgs/msg/JointState`) - The joint state updates to the robot poses. The RobotStatePublisher class takes these updates, does transformations (such as mimic joints), and then publishes the results on the tf2 topics.

Parameters
----------
* `robot_description` (string) - The original description of the robot in URDF form. This *must* be set at robot_model_server startup time. Published on the `robot_description` topic at startup.
* `publish_frequency` (double) - The maximum frequency at which non-static transforms (e.g. joint states) will be published to `/tf`. Defaults to 20.0 Hz.
* `ignore_timestamp` (bool) - Whether to accept all joint states no matter what the timestamp (true), or to only publish joint state updates if they are newer than the last publish_frequency (false). Defaults to false.
* `frame_prefix` (string) - An arbitrary prefix to add to the published tf2 frames. Defaults to the empty string.