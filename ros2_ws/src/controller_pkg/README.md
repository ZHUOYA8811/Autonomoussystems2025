# Controller

This package implements a **nonlinear SE(3) geometric controller** for a quadrotor UAV.  
It converts high-level trajectory commands into low-level rotor speed commands and serves as the final control layer before actuation.

---

## Control Architecture

The controller consists of:

* **Trajectory Tracking**:  
  Receives desired position, velocity, acceleration, and yaw from the trajectory command.

* **State Feedback**:  
  Reads current position, velocity, orientation, and angular velocity from the state estimator.

* **Translational Control**:  
  Computes position and velocity errors.  
  Applies PD feedback with gravity compensation and feedforward acceleration to determine the required total force.

* **Attitude Control**:  
  Constructs a desired rotation matrix from:
  - Force direction
  - Desired yaw angle  
  Computes rotation and angular velocity errors on SO(3).

* **Control Allocation**:  
  Converts total thrust and body moments into four rotor angular velocities using a quadrotor allocation matrix.

* **Health Monitoring**:  
  Periodically publishes a node health signal for system supervision by the state machine.

---

## ROS 2 Interfaces

### Subscribed Topics

| Topic | Message Type | Description |
| :--- | :--- | :--- |
| `command/trajectory` | `trajectory_msgs::msg::MultiDOFJointTrajectory` | Desired position, velocity, acceleration, and orientation. |
| `current_state_est` | `nav_msgs::msg::Odometry` | Current UAV state estimate. |

---

### Published Topics

| Topic | Message Type | Description |
| :--- | :--- | :--- |
| `rotor_speed_cmds` | `mav_msgs::msg::Actuators` | Angular velocities for the four rotors. |
| `statemachine/node_health` | `state_machine::msg::Answer` | Controller status heartbeat message. |

---

## Tunable Parameters

The controller exposes four main gain parameters:

- `kx` — Position gain  
- `kv` — Velocity gain  
- `kr` — Rotation gain  
- `komega` — Angular velocity gain  

These parameters can be configured in `controller_params.yaml`.

---

## Launch

### RUN
```bash
ros2 launch controller_pkg controller.launch.py