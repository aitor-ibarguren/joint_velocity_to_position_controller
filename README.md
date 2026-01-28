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
- Includes a maximum acceleration value to smooth velocioty transitions.
- Allows an **open-loop** mode in which the previously commanded joint positions are used instead of the joint positions from the state interfaces, avoiding the injection of hardware feedback latency and transport delays into the command generation loop.

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
    feedback: true
```

## License

The *joint_velocity_to_position_controller* repository has an Apache 2.0 license, as found in the [LICENSE](LICENSE) file.