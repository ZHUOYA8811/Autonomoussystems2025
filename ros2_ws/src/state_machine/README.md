# Drone Mission State Machine

This package implements a **Finite State Machine (FSM)** to oversee an autonomous drone mission. It coordinates takeoff, long-distance transit, autonomous exploration for "lanterns," and a safe return-to-home sequence.

---

## Mission Logic Flow

The FSM transitions through the following stages:

* **WAITING**: Monitors the health of `controller` and `sampler` nodes. Transitions to `TAKEOFF` once all nodes are online.
* **TAKEOFF**: Commands a vertical ascent (default 5.0m) at the current starting position.
* **TRAVELLING**: Transits to a hardcoded mission zone at `[-320.0, 10.0, 15.0]`.
* **EXPLORING**: Activates the `planner` for autonomous search. Tracks unique "lantern" detections using 3D Euclidean distance de-duplication.
* **RETURN_HOME**: Once 5 lanterns are detected, commands a return to the launch coordinates `[-38.0, 10.0, 8.0]`.
* **LAND**: Performs a final descent to `Z=0` upon arriving home.
* **DONE**: Emergency brakes (`HOLD`) applied; mission successfully concluded.
* **ERROR**: Emergency `HOLD` state triggered if any critical node goes offline during flight.


---

## ROS 2 Interfaces

### Subscribed Topics
| Topic | Message Type | Description |
| :--- | :--- | :--- |
| `statemachine/node_health` | `state_machine::msg::Answer` | Heartbeat and status from monitored nodes. |
| `current_state_est` | `nav_msgs::msg::Odometry` | Real-time position of the UAV. |
| `/detected_points` | `geometry_msgs::msg::PointStamped` | Incoming 3D coordinates of detected lanterns. |

### Published Topics
| Topic | Message Type | Description |
| :--- | :--- | :--- |
| `statemachine/state` | `std_msgs::msg::String` | Broadcasts the current FSM state for debugging. |
| `statemachine/cmd` | `state_machine::msg::Command` | High-level commands sent to `controller`, `sampler`, or `planner`. |

---

## Getting Started

### RUN
```bash
ros2 launch state_machine state_machine.launch.py