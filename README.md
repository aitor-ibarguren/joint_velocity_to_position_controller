# Joint Velocity To Position Controller

<p>
  <a href="https://github.com/aitor-ibarguren/joint_velocity_to_position_controller/actions/workflows/ros2_rolling_ci.yml">
    <img src="https://github.com/aitor-ibarguren/joint_velocity_to_position_controller/actions/workflows/ros2_rolling_ci.yml/badge.svg" alt="Build">
  </a>
  <a href="https://github.com/aitor-ibarguren/joint_velocity_to_position_controller/actions/workflows/ros2_jazzy_ci.yml">
    <img src="https://github.com/aitor-ibarguren/joint_velocity_to_position_controller/actions/workflows/ros2_jazzy_ci.yml/badge.svg" alt="Build">
  </a>
</p>

The `position_controllers/JointVelocityToPositionController` is a ROS 2 controller designed to receive joint velocity commands and convert them to joint positions. This controller extendens the `velocity_controllers/JointGroupVelocityController` controller while exposing a position command interface to the hardware. This allows robots with position-controlled drivers to be commanded using joint velocity inputs, with smooth and continuous motion generation.

## Features

- Accepts joint velocity commands as `std_msgs::msg::Float64MultiArray`.
- Computes joint positions based on the joint velocity commands.
- Includes a maximum acceleration value to smooth velocity transitions.
- Allows an **open-loop** mode in which the previously commanded joint positions are used instead of the joint positions from the state interfaces, avoiding the injection of hardware feedback latency and transport delays into the command generation loop.
- The controller includes the option to **enable a feedback topic** where the controller’s state and data is published. Specifically, the feedback includes the configured joints’ position and velocities, as well as an additional option to publish the Cartesian pose of the kinematic chain defined by the controller joints. This feedback topic, implemented as `std_msgs::msg::Float64MultiArray` and published as `controller_ns/feedback`, is designed to enable the use of the controller by external ROS2 applications that generate joint velocity commands based on joint positions, velocities, and optional Cartesian poses (e.g., a Reinforcement Learning policy generating joint velocity actions).

## Configuration

The next lines show a snippet of the *YAML* file defining the configuration of the `position_controllers/JointVelocityToPositionController` controller for a UR16e robot:

```yaml
controller_manager:
  ros__parameters:
    use_sim_time: true
    update_rate: 100  # Hz

    joint_velocity_to_position_controller:
      type: position_controllers/JointVelocityToPositionController

joint_velocity_to_position_controller:
  ros__parameters:
    joints:
      - ur_shoulder_pan_joint
      - ur_shoulder_lift_joint
      - ur_elbow_joint
      - ur_wrist_1_joint
      - ur_wrist_2_joint
      - ur_wrist_3_joint
    command_interfaces:
      - position
    state_interfaces:
      - position
      - velocity
    max_acceleration: 0.5
    open_loop: true
    feedback:
      active: true
      cartesian_pose: true
      base_link: ur_base_link
      tip_link: ur_tool0
```

Besides the typical *joints*, *command_interfaces*, and *command_interfaces*, the controller includes the next parameters:

* **max_acceleration:** Maximum acceleration, allowing a smooth transition in abrupt velocity changes.
* **open_loop:** Enable an open-loop control (joint positions from the previous update call used to calculate the commands).
* **feedback:**
  * **active:** Enable the feedback topic on `controller_ns/feedback` as `std_msgs::msg::Float64MultiArray`. By default, the controller publishes the joint positions and joint velocities (e.g., 12 values, 6 joint positions, and 6 joint velocities afterward).
  * **cartesian_pose:** Enable the Cartesian pose of the kinematic chain defined through the joints list. 
  * **base_link:** Base link of the kinematic chain used for the Cartesian pose feedback.
  * **tip_link:** Base link of the kinematic chain used for the Cartesian pose feedback.

> **⚠️ Important:** When the **cartesian_pose** is enabled, the controller verifies that the kinematic chain between the base and tip link includes all the joints of the list and in the same order. If there are discrepancies, the configuration of the controller fails.

## License

The *joint_velocity_to_position_controller* repository has an Apache 2.0 license, as found in the [LICENSE](LICENSE) file.