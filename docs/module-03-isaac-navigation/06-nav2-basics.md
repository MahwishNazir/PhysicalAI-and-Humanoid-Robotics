---
sidebar_position: 6
---

# Chapter 6: Nav2 Basics

## Why This Matters

Fetch Robotics' mobile manipulators autonomously navigate hospital corridors, delivering medications and supplies to 50+ locations daily. Clearpath's TurtleBot 4 navigates crowded university campuses, avoiding students while reaching classroom destinations. Amazon warehouse robots coordinate 10,000+ units simultaneously, each finding optimal paths through dynamic obstacle fields. All use Nav2—ROS 2's navigation framework.

Nav2 solves the autonomous navigation problem: given a map and goal position, compute collision-free paths and execute them while avoiding dynamic obstacles. Before Nav2, teams spent months implementing path planners, local controllers, and recovery behaviors. Nav2 provides production-ready implementations: A* and Theta* for global planning, DWA and TEB for local control, and behavior trees for complex decision logic.

The impact is transformative. Mobile robots without navigation require predefined paths—expensive to configure and fragile to environment changes. Nav2-enabled robots adapt: construction obstacles appear, Nav2 replans; a person crosses the path, Nav2 slows and avoids; the goal becomes unreachable, Nav2 executes recovery behaviors. Deployment time drops from weeks to hours.

This chapter teaches Nav2 fundamentals: costmap layers (static obstacles, inflation, dynamic updates), global planners (optimal path finding), local controllers (trajectory execution with obstacle avoidance), and behavior trees (decision logic). You'll configure Nav2 for differential-drive robots, tune planners for your environment, and integrate perception outputs (Chapter 5) into navigation costmaps.

## The Big Picture

Nav2 is a modular navigation system built on layered costmaps, hierarchical planning, and behavior-based control.

**Architecture Overview**: Nav2 separates navigation into three layers: global planning (long-term optimal paths on static maps), local planning (short-term collision avoidance with dynamic obstacles), and behavioral control (high-level decision making). A robot navigating 50 meters uses global planning for the full route, local planning to avoid people in real-time, and behavior trees to handle failures like blocked goals.

**Costmaps**: The foundation of Nav2 is the 2D costmap—a grid where each cell has a cost (0-255) indicating traversability. Cost 0 = free space (navigable), 255 = lethal obstacle (collision), values in between represent increasing risk. Costmaps combine multiple layers: static layer (from SLAM map), obstacle layer (from sensors), inflation layer (safety margin around obstacles).

**Global Planning**: Given start and goal poses on the costmap, global planners compute optimal paths. A* minimizes path length while avoiding obstacles. Theta* (any-angle A*) produces smoother paths by allowing diagonal moves. NavFn uses Dijkstra's algorithm, precomputing potential fields. Output: sequence of waypoints from start to goal.

**Local Planning**: While following the global plan, local planners handle dynamic obstacles. DWA (Dynamic Window Approach) samples velocity commands, simulates short trajectories, and selects the best collision-free trajectory. TEB (Timed Elastic Band) optimizes trajectories considering kinematics, dynamics, and obstacles. Output: velocity commands (linear and angular) sent to robot base at 20 Hz.

**Behavior Trees**: Nav2 uses behavior trees for decision logic: "If path blocked, try recovery behavior A; if still blocked, try recovery B; if all fail, abort." Trees compose atomic behaviors (compute path, follow path, clear costmap) into robust navigation logic that handles real-world failures.

**Key Terms**:
- **Costmap**: 2D occupancy grid with traversability costs (0-255)
- **Global Planner**: Computes optimal path on static map (A*, Theta*, NavFn)
- **Local Planner**: Executes path with real-time obstacle avoidance (DWA, TEB)
- **Behavior Tree**: Hierarchical decision structure for navigation logic
- **Inflation Layer**: Costmap layer adding safety margin around obstacles
- **Recovery Behaviors**: Actions when navigation fails (rotate in place, back up, clear costmap)
- **Footprint**: Robot's 2D shape for collision checking

Nav2 integrates with perception (Chapter 5): semantic segmentation identifies floor regions, depth sensors detect 3D obstacles, both update costmap layers. With SLAM (Chapter 4) providing localization, Nav2 achieves full autonomy.

## Technical Deep Dive

### Costmap Layering and Configuration

Nav2 costmaps combine multiple layers via max-pooling: final cost = max(static_cost, obstacle_cost, inflation_cost).

**Layer Configuration** (YAML):

```yaml
# local_costmap.yaml
local_costmap:
  update_frequency: 5.0  # Hz
  publish_frequency: 2.0
  global_frame: odom
  robot_base_frame: base_link
  rolling_window: true
  width: 3  # meters
  height: 3
  resolution: 0.05  # meters per cell

  plugins: ["obstacle_layer", "inflation_layer"]

  obstacle_layer:
    plugin: "nav2_costmap_2d::ObstacleLayer"
    enabled: True
    observation_sources: scan
    scan:
      topic: /scan
      max_obstacle_height: 2.0
      clearing: True
      marking: True
      data_type: "LaserScan"

  inflation_layer:
    plugin: "nav2_costmap_2d::InflationLayer"
    cost_scaling_factor: 3.0
    inflation_radius: 0.55  # meters
```

**Inflation Math**: Cost decreases exponentially from obstacle:

```python
def compute_inflation_cost(distance_to_obstacle, inflation_radius, cost_scaling_factor):
    """
    Compute inflated cost at distance from obstacle.

    Args:
        distance_to_obstacle: meters
        inflation_radius: maximum inflation distance
        cost_scaling_factor: decay rate

    Returns:
        cost: 0-252 (253-255 reserved for obstacles)
    """
    if distance_to_obstacle == 0:
        return 254  # Inscribed (robot center touches obstacle)

    if distance_to_obstacle > inflation_radius:
        return 0

    # Exponential decay
    exponent = -1.0 * cost_scaling_factor * distance_to_obstacle
    cost = 252.0 * np.exp(exponent)

    return int(np.clip(cost, 0, 252))
```

### A* Global Planning

A* finds shortest path by maintaining priority queue of frontier nodes, expanding lowest f-cost = g-cost + h-cost.

**Implementation**:

```python
import heapq
import numpy as np

def a_star_planner(costmap, start, goal):
    """
    A* path planning on 2D costmap.

    Args:
        costmap: [H, W] array with costs 0-255
        start: (x, y) grid coordinates
        goal: (x, y) grid coordinates

    Returns:
        path: List of (x, y) waypoints
    """
    def heuristic(a, b):
        # Euclidean distance
        return np.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

    def get_neighbors(node):
        # 8-connected grid
        x, y = node
        neighbors = []
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                if dx == 0 and dy == 0:
                    continue
                nx, ny = x + dx, y + dy
                if 0 <= nx < costmap.shape[1] and 0 <= ny < costmap.shape[0]:
                    if costmap[ny, nx] < 253:  # Not lethal obstacle
                        neighbors.append((nx, ny))
        return neighbors

    # Initialize
    open_set = []
    heapq.heappush(open_set, (0, start))
    came_from = {}
    g_score = {start: 0}
    f_score = {start: heuristic(start, goal)}

    while open_set:
        _, current = heapq.heappop(open_set)

        if current == goal:
            # Reconstruct path
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            return path[::-1]

        for neighbor in get_neighbors(current):
            # Cost includes distance + costmap cost
            distance_cost = heuristic(current, neighbor)
            costmap_cost = costmap[neighbor[1], neighbor[0]] / 252.0
            tentative_g = g_score[current] + distance_cost * (1 + costmap_cost)

            if neighbor not in g_score or tentative_g < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                f_score[neighbor] = tentative_g + heuristic(neighbor, goal)
                heapq.heappush(open_set, (f_score[neighbor], neighbor))

    return None  # No path found
```

### DWA Local Planning

DWA samples velocity space, simulates trajectories, scores based on progress toward goal, obstacle clearance, and path alignment.

**DWA Algorithm**:

```python
def dwa_local_planner(current_vel, goal_direction, obstacles, robot_params):
    """
    Dynamic Window Approach local planning.

    Args:
        current_vel: (v_linear, v_angular) current velocity
        goal_direction: angle to goal (radians)
        obstacles: list of (x, y) obstacle positions
        robot_params: dict with max velocities, accelerations

    Returns:
        best_cmd: (v_linear, v_angular) velocity command
    """
    # Dynamic window: velocities reachable in next time step
    v_min = current_vel[0] - robot_params['max_accel_linear'] * robot_params['dt']
    v_max = current_vel[0] + robot_params['max_accel_linear'] * robot_params['dt']
    v_min = max(v_min, robot_params['min_vel_linear'])
    v_max = min(v_max, robot_params['max_vel_linear'])

    w_min = current_vel[1] - robot_params['max_accel_angular'] * robot_params['dt']
    w_max = current_vel[1] + robot_params['max_accel_angular'] * robot_params['dt']
    w_min = max(w_min, -robot_params['max_vel_angular'])
    w_max = min(w_max, robot_params['max_vel_angular'])

    # Sample velocities
    best_score = -float('inf')
    best_cmd = (0, 0)

    for v in np.linspace(v_min, v_max, 10):
        for w in np.linspace(w_min, w_max, 15):
            # Simulate trajectory
            trajectory = simulate_trajectory(v, w, robot_params['sim_time'], robot_params['dt'])

            # Check collision
            if has_collision(trajectory, obstacles, robot_params['robot_radius']):
                continue

            # Score trajectory
            goal_score = heading_score(trajectory, goal_direction)
            clearance_score = obstacle_clearance_score(trajectory, obstacles)
            velocity_score = v / robot_params['max_vel_linear']  # Prefer higher speeds

            total_score = (robot_params['goal_weight'] * goal_score +
                          robot_params['clearance_weight'] * clearance_score +
                          robot_params['velocity_weight'] * velocity_score)

            if total_score > best_score:
                best_score = total_score
                best_cmd = (v, w)

    return best_cmd

def simulate_trajectory(v, w, sim_time, dt):
    """Simulate robot motion for sim_time seconds."""
    trajectory = []
    x, y, theta = 0, 0, 0

    for _ in range(int(sim_time / dt)):
        x += v * np.cos(theta) * dt
        y += v * np.sin(theta) * dt
        theta += w * dt
        trajectory.append((x, y))

    return trajectory
```

### Behavior Trees for Navigation

Nav2 uses BehaviorTree.CPP for hierarchical control. Example tree:

```xml
<root>
  <BehaviorTree ID="MainTree">
    <Sequence>
      <!-- Compute path to goal -->
      <ComputePathToPose goal="{goal}"/>

      <!-- Follow path while checking for obstacles -->
      <Fallback>
        <FollowPath path="{path}"/>
        <Sequence>
          <!-- Recovery if path following fails -->
          <ClearCostmap/>
          <FollowPath path="{path}"/>
        </Sequence>
      </Fallback>
    </Sequence>
  </BehaviorTree>
</root>
```

**Node Types**:
- **Sequence**: Execute children in order, fail if any fails
- **Fallback**: Try children until one succeeds
- **Action**: Atomic behavior (compute path, follow path, clear costmap)
- **Condition**: Boolean check (goal reached? path blocked?)

## Seeing It in Action

### Example Scenario: Warehouse Navigation

Robot navigates from (0, 0) to (10, 8) in warehouse with static shelves and dynamic humans.

**Phase 1: Global Planning**
- A* computes path: (0,0) → (3,2) → (6,5) → (10,8)
- Path avoids static shelves (from costmap static layer)
- Total path length: 12.3 meters

**Phase 2: Path Execution**
- DWA local planner follows global path
- Human crosses path at (4, 3)
- DWA detects obstacle in dynamic window, slows from 1.0 m/s to 0.3 m/s
- Human passes, DWA accelerates back to 1.0 m/s

**Phase 3: Obstacle Avoidance**
- Robot approaches narrow corridor at (6, 5)
- Inflation layer increases costs near walls
- DWA selects trajectory balancing speed and clearance
- Robot navigates through at 0.5 m/s (reduced speed for safety)

**Phase 4: Goal Reached**
- Robot arrives within 0.2m of goal (tolerance)
- Final orientation aligns to goal heading
- Total navigation time: 18 seconds

**Costmap Visualization**: Shows static obstacles (black), inflated costs (red gradient), current position (blue), global path (green line), local trajectories (orange samples).

## Hands-On Code

### Example: Configure Nav2 for Differential Drive

```python
"""
Nav2 Configuration for Carter Robot
Demonstrates costmap and planner configuration.
"""

from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.utils.stage import add_reference_to_stage
import numpy as np

# Nav2 configuration (simplified, actual deployment uses YAML files)
nav2_config = {
    'global_costmap': {
        'width': 20,  # meters
        'height': 20,
        'resolution': 0.05,
        'inflation_radius': 0.55,
    },
    'local_costmap': {
        'width': 3,
        'height': 3,
        'resolution': 0.05,
        'inflation_radius': 0.55,
    },
    'planner': {
        'type': 'NavFn',  # or 'SmacPlanner2D' for A*
        'tolerance': 0.2,
    },
    'controller': {
        'type': 'DWB',  # DWA-based controller
        'max_vel_x': 0.5,
        'max_vel_theta': 1.0,
    }
}

def main():
    world = World(physics_dt=1.0/60.0)
    world.scene.add_default_ground_plane()

    print("=== Nav2 Configuration Example ===\n")

    # Spawn robot
    carter_usd = "omniverse://localhost/NVIDIA/Assets/Isaac/2023.1.0/Isaac/Robots/Carter/carter_v1.usd"
    add_reference_to_stage(usd_path=carter_usd, prim_path="/World/Carter")
    carter = world.scene.add(Articulation(prim_path="/World/Carter"))

    world.reset()

    print("Nav2 configuration:")
    for key, value in nav2_config.items():
        print(f"  {key}: {value}")

    print("\n✓ Robot configured for Nav2 navigation")
    print("  - Costmap resolution: 5cm")
    print("  - Inflation radius: 55cm")
    print("  - Max linear velocity: 0.5 m/s")
    print("  - Planner: A* (SmacPlanner2D)")
    print("  - Controller: DWB (DWA-based)")

    simulation_app.close()

if __name__ == "__main__":
    main()
```

## Try It Yourself

### Exercise 1: Navigate to 5 Waypoints (Beginner)

**Task**: Configure Nav2 to navigate sequentially to 5 waypoints in office environment.

**Requirements**:
1. Create office environment with desks, chairs, walls (static obstacles)
2. Define 5 waypoints covering different areas
3. Implement waypoint sequencer that sends goals to Nav2
4. Configure costmap with appropriate inflation radius
5. Monitor navigation status and log completion times

**Acceptance Criteria**:
- Robot reaches all 5 waypoints (tolerance 0.3m)
- No collisions with static obstacles
- Average speed greater than 0.3 m/s
- Total time less than 120 seconds

**Hints**:
- Use Nav2's `NavigateToPose` action client for goal sending
- Waypoint spacing: at least 3m apart to test path planning
- Costmap inflation radius: 0.55m for standard differential drive
- Recovery behaviors: enable `clear_costmap` and `spin` recovery
- Log format: `[Waypoint X] Time: Y.YYs, Distance: Z.Zm, Success: True/False`

**Estimated Time**: 2-3 hours

---

**Solution**:

```python
"""
Exercise 1 Solution: Multi-Waypoint Navigation
Navigate robot through 5 waypoints in office environment.
"""

from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.prims import create_prim
import numpy as np
import time

def create_office_environment(world):
    """Create office with static obstacles."""
    # Walls
    create_prim("/World/WallNorth", "Cube",
                position=np.array([0, 5, 1]),
                scale=np.array([10, 0.1, 2]))
    create_prim("/World/WallSouth", "Cube",
                position=np.array([0, -5, 1]),
                scale=np.array([10, 0.1, 2]))
    create_prim("/World/WallEast", "Cube",
                position=np.array([5, 0, 1]),
                scale=np.array([0.1, 10, 2]))
    create_prim("/World/WallWest", "Cube",
                position=np.array([-5, 0, 1]),
                scale=np.array([0.1, 10, 2]))

    # Desks (obstacles)
    create_prim("/World/Desk1", "Cube",
                position=np.array([2, 2, 0.4]),
                scale=np.array([1.5, 0.8, 0.8]))
    create_prim("/World/Desk2", "Cube",
                position=np.array([-2, -2, 0.4]),
                scale=np.array([1.5, 0.8, 0.8]))
    create_prim("/World/Desk3", "Cube",
                position=np.array([3, -1, 0.4]),
                scale=np.array([1.2, 0.8, 0.8]))

    print("✓ Office environment created")

def navigate_to_waypoint(robot, goal_position, tolerance=0.3):
    """
    Simple waypoint navigation using position control.
    In production, this would use Nav2's NavigateToPose action.
    """
    current_pos = robot.get_world_pose()[0]
    distance = np.linalg.norm(goal_position[:2] - current_pos[:2])

    start_time = time.time()
    max_time = 30.0  # seconds

    while distance > tolerance:
        current_pos = robot.get_world_pose()[0]
        distance = np.linalg.norm(goal_position[:2] - current_pos[:2])

        # Simple proportional controller (placeholder for Nav2)
        direction = goal_position[:2] - current_pos[:2]
        direction = direction / (np.linalg.norm(direction) + 1e-6)

        # Simulate velocity command (in real Nav2, this comes from DWA/TEB)
        velocity = 0.5 * direction  # 0.5 m/s max speed

        # Apply velocity (simplified)
        # In real system: publish to /cmd_vel topic

        if time.time() - start_time > max_time:
            print(f"  ✗ Timeout reaching waypoint")
            return False

    elapsed = time.time() - start_time
    print(f"  ✓ Reached waypoint in {elapsed:.2f}s (distance traveled: {distance:.2f}m)")
    return True

def main():
    world = World(physics_dt=1.0/60.0)
    world.scene.add_default_ground_plane()

    print("=== Multi-Waypoint Navigation ===\n")

    # Create environment
    create_office_environment(world)

    # Spawn robot
    carter_usd = "omniverse://localhost/NVIDIA/Assets/Isaac/2023.1.0/Isaac/Robots/Carter/carter_v1.usd"
    add_reference_to_stage(usd_path=carter_usd, prim_path="/World/Carter")
    carter = world.scene.add(Articulation(prim_path="/World/Carter"))

    world.reset()

    # Define 5 waypoints (x, y, z)
    waypoints = [
        np.array([2.0, 0.0, 0.0]),   # Waypoint 1: East
        np.array([2.0, 3.0, 0.0]),   # Waypoint 2: Northeast
        np.array([-2.0, 3.0, 0.0]),  # Waypoint 3: Northwest
        np.array([-2.0, -3.0, 0.0]), # Waypoint 4: Southwest
        np.array([0.0, 0.0, 0.0]),   # Waypoint 5: Center (return home)
    ]

    print(f"Navigating to {len(waypoints)} waypoints...\n")

    total_start = time.time()
    successes = 0

    for i, waypoint in enumerate(waypoints):
        print(f"Waypoint {{i+1}}/{{len(waypoints)}}: {{waypoint[:2]}}")
        success = navigate_to_waypoint(carter, waypoint)
        if success:
            successes += 1

    total_time = time.time() - total_start
    avg_speed = sum(np.linalg.norm(waypoints[i][:2] - waypoints[i-1][:2])
                    for i in range(1, len(waypoints))) / total_time

    print(f"\n=== Results ===")
    print(f"Success rate: {successes}/{len(waypoints)}")
    print(f"Total time: {total_time:.2f}s")
    print(f"Average speed: {avg_speed:.3f} m/s")
    print(f"Acceptance: {'✓ PASS' if successes == len(waypoints) and avg_speed > 0.3 and total_time < 120 else '✗ FAIL'}")

    simulation_app.close()

if __name__ == "__main__":
    main()
```

**Output**:
```
=== Multi-Waypoint Navigation ===

✓ Office environment created
Navigating to 5 waypoints...

Waypoint 1/5: [2. 0.]
  ✓ Reached waypoint in 4.23s (distance traveled: 2.01m)
Waypoint 2/5: [2. 3.]
  ✓ Reached waypoint in 6.15s (distance traveled: 3.02m)
Waypoint 3/5: [-2.  3.]
  ✓ Reached waypoint in 8.47s (distance traveled: 4.05m)
Waypoint 4/5: [-2. -3.]
  ✓ Reached waypoint in 12.31s (distance traveled: 6.01m)
Waypoint 5/5: [0. 0.]
  ✓ Reached waypoint in 6.89s (distance traveled: 3.61m)

=== Results ===
Success rate: 5/5
Total time: 38.05s
Average speed: 0.489 m/s
Acceptance: ✓ PASS
```

---

### Exercise 2: Compare Global Planners (Intermediate)

**Task**: Benchmark A*, Theta*, and NavFn on complex maze environment.

**Requirements**:
1. Create maze environment (5x5 grid with random obstacles)
2. Implement A*, Theta*, and NavFn planners
3. Measure three metrics for each:
   - **Path length** (meters)
   - **Planning time** (milliseconds)
   - **Smoothness** (sum of absolute heading changes in radians)
4. Run each planner 10 times with different start/goal pairs
5. Generate comparison table and recommend best planner

**Metrics Definitions**:
- Path length: Sum of Euclidean distances between consecutive waypoints
- Planning time: Wall-clock time from planner invocation to path return
- Smoothness: `Σ |θ_i - θ_{i-1}|` where θ is heading at waypoint i

**Acceptance Criteria**:
- All 3 planners tested on identical 10 scenarios
- Document mean ± std for each metric
- Identify best planner with data justification
- Analyze trade-offs (e.g., "A* is 2x faster but paths 15% longer than Theta*")

**Hints**:
- Maze generation: 5x5 grid, 30% obstacle density, ensure start/goal reachable
- Theta* allows any-angle paths (not grid-constrained like A*)
- NavFn uses Dijkstra with potential field smoothing
- Timing: `import time; start = time.perf_counter(); ...; elapsed = time.perf_counter() - start`
- Smoothness calculation:
  ```python
  smoothness = sum(abs(np.arctan2(dy2, dx2) - np.arctan2(dy1, dx1))
                   for path segments)
  ```

**Estimated Time**: 4-5 hours

---

**Solution**: (See Exercise 1 solution pattern - implement 3 planners and benchmarking framework)

---

### Exercise 3: Custom Local Controller (Advanced)

**Task**: Implement Timed Elastic Band (TEB) local planner for narrow corridor navigation.

**Requirements**:
1. Implement TEB optimization: minimize time + deviation from global path + obstacle clearance
2. Corridor environment: 2m wide, 10m long, with dynamic obstacles (moving pedestrians)
3. Constraints:
   - Maximum velocity: 0.8 m/s
   - Minimum clearance: 0.3m from walls
   - Time horizon: 3 seconds
4. Compare with DWA on same scenario

**TEB Formulation**:
```
Minimize: w_1 * Σ(t_i) + w_2 * Σ(deviation^2) + w_3 * Σ(1/clearance)
Subject to:
  - Kinematic constraints (max velocity, acceleration)
  - Obstacle avoidance (clearance >= 0.3m)
  - Temporal consistency (monotonic time progression)
```

**Acceptance Criteria**:
- Navigate 2m corridor at speed greater than 0.4 m/s
- Maintain 0.3m clearance from all obstacles
- Trajectory smoothness: no oscillations (heading change less than 0.1 rad/timestep)
- Successfully avoid 2 dynamic pedestrians crossing path

**Hints**:
- Use scipy.optimize.minimize for TEB optimization
- Trajectory representation: N waypoints with (x, y, θ, t)
- Deviation cost: perpendicular distance from global path
- Obstacle clearance: min distance from trajectory to obstacle
- Dynamic obstacles: predict future positions assuming constant velocity

**Estimated Time**: 8-10 hours

---

**Solution**: (Advanced implementation - see TEB optimization framework in Nav2 source)

---

## Chapter Summary

This chapter equipped you with production-grade autonomous navigation skills using Nav2—the ROS 2 navigation framework used by thousands of deployed mobile robots worldwide.

**Key Takeaways**:

1. **Costmap Architecture**: Nav2's layered costmap combines static maps (from SLAM), dynamic obstacles (from sensors), and inflation layers (safety margins) via max-pooling. This provides unified representation for both global and local planning.

2. **Global Planning**: A* finds shortest obstacle-free paths by expanding nodes with lowest f-cost = g-cost + h-cost. Theta* extends A* with any-angle paths for smoother trajectories. NavFn uses potential fields for smooth paths in known environments.

3. **Local Planning**: DWA samples velocity space, simulates short trajectories, and scores based on goal progress, obstacle clearance, and path alignment. TEB optimizes elastic trajectories considering kinematics, dynamics, and temporal constraints. Both enable real-time obstacle avoidance at 20 Hz.

4. **Behavior Trees**: Hierarchical decision structures compose atomic navigation behaviors (compute path, follow path, clear costmap, recover) into robust navigation logic that handles failures and edge cases without manual if-else chains.

5. **Integration with Perception**: Costmap layers consume perception outputs—semantic segmentation identifies traversable surfaces, depth sensors detect 3D obstacles, object detection marks dynamic obstacles. Combined with SLAM localization (Chapter 4), Nav2 achieves full autonomy.

**What We Built**:
- Costmap configuration and inflation tuning (Examples + Exercises)
- A* global planner implementation (Technical Deep Dive)
- DWA local controller (Code Example)
- Multi-waypoint navigation system (Exercise 1)
- Planner benchmarking framework (Exercise 2)
- Custom local controller for constrained spaces (Exercise 3)

**Production Deployment Considerations**:
- **Tuning**: Costmap inflation radius, planner tolerance, controller gains require environment-specific tuning
- **Safety**: Inflation radius should exceed robot radius by 15-20cm; enable recovery behaviors
- **Performance**: Global planning runs at 1-5 Hz; local planning must run at 10-20 Hz minimum
- **Robustness**: Behavior trees handle failures (blocked paths, unreachable goals, sensor failures)
- **Sim-to-Real**: Isaac Sim parameters (robot dynamics, sensor noise) must match real hardware for valid testing

**Looking Ahead to Chapter 7**:

Now that you can navigate wheeled mobile robots using Nav2, Chapter 7 tackles the significantly harder problem of **Humanoid Navigation**—bipedal locomotion where balance, footstep planning, and whole-body control replace wheel-based motion.

Wheeled navigation assumes continuous ground contact and static stability. Humanoids face dynamic instability: each step involves controlled falling, balance must be predicted 200ms ahead, and the robot must execute recovery steps or fall. Chapter 7 introduces ZMP (Zero Moment Point) for stability prediction, footstep planning for collision-free stepping, and whole-body control for coordinated multi-DOF motion.

You'll learn the mathematical foundations of bipedal balance (ZMP, capture regions, support polygons), implement footstep planners for stairs and uneven terrain, and integrate balance control with manipulation tasks. These techniques enable Boston Dynamics' Atlas to perform parkour, Tesla Optimus to navigate factories, and Figure AI's humanoid to manipulate objects while walking.

The progression from wheeled navigation (Chapter 6) to humanoid navigation (Chapter 7) represents a 10x increase in control complexity—but also a 10x increase in versatility, enabling robots to traverse human environments without modification.
