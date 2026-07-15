Robot Model Server
==================

Robot Model Server serves the same purpose as `robot_state_publisher` --
publish robot model and transforms, but has a few distinctive features:

- modular design;
- cumulative inertial parameters;
- visual representations of inertial parameters.


Modules
-------

Repository contains three packages:

* **`robot_model_server_core`** — core library with minimal ROS dependencies.

* **`robot_model_server_ros`** — ROS 2 wrapper for the core library, deals with
  ROS topics and parameters.

* **`robot_model_server_tests`** — tests and examples.


Cumulative inertial parameters
------------------------------

Core library provides functionality for computation of cumulative inertial
parameters: mass, center of mass position, and 3d inertia matrix. Those can be
useful in the cases where model represents a robot with no movable joints or
their motion can be neglected, e.g., quadcopters.


Visualization of inertial parameters
------------------------------------

Robot model server can optionally generate visual representations of inertial
parameters and publish them as visualization markers to `rviz`. This
functionality is almost identical to the one provided by default `rviz` robot
model plugin, but gives more control to the user and includes cumulative
inertial parameters.


ROS node parameters
-------------------

* `model.description` (string) - The original description of the robot in URDF
  form.
* `model.description_file` (string) - Read model description from a file,
  mutually exclusive with `model.description`
* `transform.frequency` (double) - The maximum frequency at which non-static
  transforms (e.g. joint states) will be published to `/tf`.
* `joint_states.ignore_timestamp` (bool) - Whether to accept all joint states
  no matter what the timestamp (true), or to only publish joint state updates
  if they are newer than the last transform.frequency (false). Defaults to
  false.
* `transform.frame_prefix` (string) - An arbitrary prefix to add to the
  published tf2 frames. Defaults to the empty string.
* `visualization.inertia` (bool) - Whether to publish inertial visualization
  markers. When `inertia.cumulative` is also true, the cumulative inertia
  marker is published on the same topic. Defaults to false.
* `visualization.alpha` (double) - Transparency of the inertia visualization
  markers. Range 0.0 (fully transparent) to 1.0 (fully opaque). Effective when
  `visualization.inertia` is true. Defaults to 0.5.
* `inertia.cumulative` (bool) - Whether to publish cumulative inertial
  parameters: the center-of-mass transform, the cumulative inertia message, and
  (when `visualization.inertia` is also true) the cumulative inertia visualization
  marker. Defaults to false.
* `inertia.tolerance` (double) - Tolerance for skipping links with negligible
  inertial parameters (mass and inertia components all below this value).
  Effective when either `visualization.inertia` or `inertia.cumulative` is true.
  Defaults to 0.0.


Topics
------

### Published

* `robot_description` - robot model description in URDF/SDF format as provided
  by the corresponding parameter.
* `tf` - the transforms corresponding to the movable joints of the robot.
* `tf_static` - The transforms corresponding to the static joints of the robot
  and center of mass position when computation of cumulative inertial
  parameters is enabled.
* `robot_model_server/cumulative_inertia` (`geometry_msgs::Inertia`) - the
  cumulative inertial parameters (mass, COM, inertia matrix) of the robot
  assuming all joints fixed at 0. Published on a latched topic when
  `inertia.cumulative` is true.

### Subscribed

* `joint_states` (`sensor_msgs/msg/JointState`) - The joint state updates to
  the robot poses.

