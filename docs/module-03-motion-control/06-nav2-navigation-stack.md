---
title: "Nav2 Navigation Stack"
sidebar_label: "Nav2 Basics"
sidebar_position: 6
description: "Path planning and obstacle avoidance fundamentals with ROS 2 Nav2"
tags: [nav2, navigation, path-planning, obstacle-avoidance, ros2]
keywords: [Nav2, navigation stack, path planning, obstacle avoidance, costmap, behavior tree]
difficulty: intermediate
estimated_time: "2 hours"
prerequisites: ["Chapter 4: Isaac ROS Visual SLAM"]
---

# Nav2 Navigation Stack

*Content coming soon. This chapter will cover:*

## Nav2 Overview

- What is Nav2 (Navigation2)?
- Architecture and components
- Difference from ROS 1 navigation
- When to use Nav2 vs custom planning
- Community and ecosystem

## Core Components

### Costmap 2D
- Layers: static, obstacle, inflation, voxel
- Global vs local costmaps
- Cost calculation
- Sensor integration (LiDAR, camera, depth)
- Configuration and tuning

### Planners

**Global Planners** (long-range paths):
- NavFn (Dijkstra-based)
- Smac Planner (State Lattice, Hybrid A*)
- ThetaStar (any-angle planning)
- Configuration and trade-offs

**Local Planners** (short-range control):
- DWB (Dynamic Window Approach)
- TEB (Timed Elastic Band)
- MPPI (Model Predictive Path Integral)
- Regulated Pure Pursuit
- Rotation Shim Controller

### Behavior Trees

- Behavior tree basics
- Default navigation behavior tree
- Recovery behaviors (backup, spin, wait)
- Custom behaviors
- Blackboard and logging

### Behavior Server

- Simple Commander API
- Goal handling
- Cancellation and preemption
- Waypoint following
- Multi-robot coordination

## Map Representation

- Occupancy grids
- Map server (static maps)
- SLAM integration (dynamic maps)
- Map saving and loading
- Multi-floor maps

## Localization

- AMCL (Adaptive Monte Carlo Localization)
- Particle filter concepts
- Integration with SLAM
- Initial pose estimation
- Kidnapped robot problem

## Configuration Files

- Navigation parameters YAML
- Costmap configuration
- Planner parameters
- Controller tuning
- Behavior tree XML

## Tuning for Your Robot

1. **Footprint Definition**: Circular, rectangular, polygon
2. **Kinematic Limits**: Max velocity, acceleration, turning radius
3. **Safety Margins**: Inflation radius, obstacle cost scaling
4. **Planning Parameters**: Resolution, tolerance, timeout
5. **Local Planner**: Trajectory scoring weights

## Advanced Features

- Keepout zones and speed limits
- Dynamic obstacles handling
- Elevation mapping (2.5D navigation)
- Time-dependent planning
- Social navigation (human-aware)

## Debugging and Visualization

- RViz2 panels and displays
- Costmap visualization
- Planned path display
- Footprint visualization
- Debugging tools and techniques

## Common Issues

- Robot gets stuck
- Oscillating behavior
- Failure to reach goal
- Poor path quality
- Computational performance

## Hands-On Projects

1. **Basic Navigation**: Send goals and navigate in a known map
2. **Obstacle Avoidance**: Dynamic obstacle response tuning
3. **Waypoint Following**: Navigate through multiple goals
4. **Recovery Behaviors**: Test and configure stuck situations

*For now, review [Nav2 Documentation](https://navigation.ros.org/).*
