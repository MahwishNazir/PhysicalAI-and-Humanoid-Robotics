# System Architecture Design

**Project**: Autonomous Navigation Robot
**Date**: 2025-12-24
**Author**: Student Name

## 1. Requirements Analysis

### Functional Requirements
1. Robot must detect obstacles using lidar sensor
2. Robot must avoid obstacles by turning away
3. Robot must move forward when path is clear
4. System must run in Gazebo simulation
5. All components must be configurable via parameters

### Non-Functional Requirements
1. Sensor processing latency < 100ms
2. Control loop frequency: 10 Hz minimum
3. System must be observable (publish state information)
4. Code must follow ROS 2 best practices

## 2. Node Responsibilities

### Node 1: `laser_processor`
- **Purpose**: Process raw laser scan data to detect obstacles
- **Inputs**:
  - `/scan` (sensor_msgs/LaserScan) - Raw lidar data from simulation
- **Outputs**:
  - `/obstacle_detected` (std_msgs/Bool) - True if obstacle within threshold
- **Parameters**:
  - `min_distance` (double, default: 0.5m) - Obstacle detection threshold
- **Logic**: Check if any laser range < min_distance

### Node 2: `obstacle_avoider`
- **Purpose**: Generate velocity commands based on obstacle state
- **Inputs**:
  - `/obstacle_detected` (std_msgs/Bool) - Obstacle detection signal
- **Outputs**:
  - `/cmd_vel` (geometry_msgs/Twist) - Velocity commands to robot
- **Parameters**:
  - `linear_speed` (double, default: 0.2 m/s) - Forward speed
  - `angular_speed` (double, default: 0.5 rad/s) - Turning speed
- **Logic**:
  - If obstacle detected: turn in place (linear=0, angular=angular_speed)
  - If path clear: move forward (linear=linear_speed, angular=0)

### Node 3: Gazebo Simulation
- **Purpose**: Provide simulated environment and robot
- **Outputs**: `/scan`, sensor data
- **Inputs**: `/cmd_vel` commands

## 3. Topic Architecture

```
┌─────────────────┐
│     Gazebo      │
│   (Simulation)  │
└────┬────────┬───┘
     │        │
     │ /scan  │ /cmd_vel
     │        │
     ▼        │
┌─────────────────┐        ┌──────────────────┐
│ laser_processor │        │ obstacle_avoider │
│                 ├───────►│                  │
└─────────────────┘        └──────────────────┘
   /obstacle_detected
```

### Topic Details

| Topic Name | Message Type | Publisher | Subscriber | QoS |
|------------|--------------|-----------|------------|-----|
| `/scan` | sensor_msgs/LaserScan | Gazebo | laser_processor | Best Effort, depth 10 |
| `/obstacle_detected` | std_msgs/Bool | laser_processor | obstacle_avoider | Reliable, depth 10 |
| `/cmd_vel` | geometry_msgs/Twist | obstacle_avoider | Gazebo | Best Effort, depth 10 |

## 4. Data Flow

1. **Simulation → Sensing**: Gazebo publishes LaserScan at 10 Hz on `/scan`
2. **Sensing → Decision**: `laser_processor` processes scans, publishes Bool on `/obstacle_detected`
3. **Decision → Control**: `obstacle_avoider` reads Bool, publishes Twist on `/cmd_vel`
4. **Control → Actuation**: Gazebo receives Twist, moves robot

## 5. State Machine (Obstacle Avoider)

```
          Path Clear
        ┌────────────┐
        │            │
        ▼            │
    ┌───────┐    ┌────────┐
    │ MOVE  │◄───│  TURN  │
    │FORWARD│    │        │
    └───────┘    └────────┘
        │            ▲
        │            │
        └────────────┘
       Obstacle Detected
```

**States**:
- **MOVE_FORWARD**: linear_x = linear_speed, angular_z = 0
- **TURN**: linear_x = 0, angular_z = angular_speed

**Transitions**:
- MOVE_FORWARD → TURN: when `/obstacle_detected` = True
- TURN → MOVE_FORWARD: when `/obstacle_detected` = False

## 6. Configuration Parameters

All parameters stored in `config/system_params.yaml`:

```yaml
laser_processor:
  ros__parameters:
    min_distance: 0.5  # meters

obstacle_avoider:
  ros__parameters:
    linear_speed: 0.2   # m/s
    angular_speed: 0.5  # rad/s
```

## 7. Launch Architecture

Master launch file `robot_system.launch.py` orchestrates:

1. **Start Gazebo** with custom world (from Chapter 7)
2. **Spawn robot** at specified position
3. **Launch laser_processor** node with parameters
4. **Launch obstacle_avoider** node with parameters
5. **Optional**: Launch RViz2 for visualization

## 8. Testing Strategy

### Unit Tests
- `laser_processor`: Publish fake LaserScan, verify correct Bool output
- `obstacle_avoider`: Publish fake Bool, verify correct Twist output

### Integration Test
- Launch full system, verify robot avoids obstacles in simulation

### Acceptance Criteria
- Robot moves forward when `>0.5m` from obstacles
- Robot turns when `<0.5m` from obstacles
- No crashes into obstacles for 2-minute test run

## 9. File Organization

```
solutions/chapter-08/autonomous-navigation-robot/
├── architecture.md          # This file
├── launch/
│   └── robot_system.launch.py
├── config/
│   └── system_params.yaml
├── src/
│   ├── laser_processor.py
│   └── obstacle_avoider.py
├── test/
│   └── test_obstacle_detection.py
└── README.md
```

## 10. Future Enhancements

Potential extensions (covered in exercises):
1. Waypoint navigation: Add goal-seeking behavior
2. Wall following: Use side laser ranges to follow walls
3. Sensor fusion: Combine camera + lidar for better detection
4. Recovery behaviors: Detect "stuck" state, execute recovery maneuvers
5. Multi-robot coordination: Multiple robots with namespace isolation

---

**Next Steps**: Implement nodes per this architecture, test individually, then integrate into complete system.
