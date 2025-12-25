---
sidebar_position: 7
---

# Chapter 7: Humanoid Navigation

## Why This Matters

Boston Dynamics' Atlas performs parkour—sprinting, jumping, and recovering from pushes while maintaining balance on two legs. Tesla Optimus navigates factory floors, stepping over obstacles and adapting gait to uneven terrain. Figure AI's humanoid manipulates objects while walking, coordinating arm motions with leg movements to maintain stability. All rely on humanoid navigation—the challenge of bipedal locomotion on dynamic surfaces.

Humanoid navigation differs fundamentally from wheeled robots. Wheeled platforms (Chapter 6) maintain continuous ground contact; stability comes for free. Humanoids face constant instability: each step involves controlled falling, dynamic balance depends on footstep timing and placement, and the robot must predict tipping 200ms in advance. A wheeled robot slows to stop; a humanoid must execute recovery steps or fall.

The impact extends beyond robotics. Humanoid navigation enables robots to traverse human environments without modification: stairs, narrow doorways, cluttered floors. Warehouses deploy humanoids for vertical picking where wheeled robots cannot reach. Disaster response teams use humanoids to navigate rubble where wheels would fail. The market for bipedal platforms is projected to exceed $10B by 2030.

This chapter teaches humanoid navigation fundamentals: ZMP (Zero Moment Point) for balance prediction, footstep planning for collision-free stepping, whole-body control for coordinated motion, and push recovery for disturbance rejection. You'll implement ZMP computation, design footstep planners for obstacles, and integrate balance control with manipulation tasks (Chapter 8).

## The Big Picture

Humanoid navigation combines balance prediction, footstep planning, and whole-body control to achieve stable bipedal locomotion.

**Architecture Overview**: Humanoid navigation operates in three layers: balance layer (ZMP computation and stability monitoring), planning layer (footstep sequence generation), and control layer (joint-space trajectory execution). A robot walking 10 meters uses balance monitoring to predict tipping, footstep planning to avoid obstacles, and whole-body control to execute coordinated leg and torso motions.

**Zero Moment Point (ZMP)**: The foundation of bipedal stability. ZMP is the point on the ground where net moment from gravity and inertia equals zero. For a humanoid to remain stable, ZMP must stay inside the support polygon (convex hull of ground contact points). Single-leg stance has small support polygon (foot outline), double-leg stance expands polygon between both feet.

**Balance Dynamics**: During walking, the robot alternates between unstable (single support) and stable (double support) phases. In single support, ZMP must stay within one foot's boundary—violations cause tipping. Controllers predict ZMP trajectory 200-500ms ahead, adjusting COM (Center of Mass) acceleration to keep ZMP inside bounds. Push disturbances require reactive footstep adjustments to expand support polygon.

**Footstep Planning**: Given goal position and obstacle map, footstep planners compute stepping stone sequences. A* searches footstep graph (nodes = foot placements, edges = kinematically valid transitions). Constraints include maximum step length (0.5m), lateral step width (0.15-0.4m), and rotation (25 degrees). Output: sequence of left/right foot poses forming collision-free path to goal.

**Whole-Body Control**: Humanoids have 20-40 DOF (degrees of freedom); walking requires coordinated control. Whole-body controllers solve inverse kinematics with task hierarchy: highest priority to balance (ZMP constraint), second priority to footstep tracking (swing foot trajectory), third priority to posture (joint limits). Quadratic programming solves for joint velocities satisfying all constraints simultaneously.

**Key Terms**:
- **ZMP (Zero Moment Point)**: Ground point where tipping moment equals zero
- **Support Polygon**: Convex hull of ground contact points
- **COM (Center of Mass)**: Weighted average position of robot mass
- **Single Support Phase**: One foot on ground (unstable)
- **Double Support Phase**: Both feet on ground (stable)
- **Footstep Graph**: Search space of valid foot placements
- **Whole-Body Controller**: Multi-task optimization for coordinated motion
- **Push Recovery**: Reactive stepping to reject disturbances

Humanoid navigation integrates with perception (Chapter 5): depth sensors detect stair geometry, terrain classification identifies compliant surfaces. With Nav2 (Chapter 6) providing high-level goals, humanoids achieve autonomous navigation in unstructured environments.

## Technical Deep Dive

### ZMP Computation and Stability Criteria

ZMP position depends on COM position, velocity, acceleration, and gravity.

**ZMP Formula** (2D simplified):

```python
import numpy as np

def compute_zmp_2d(com_pos, com_vel, com_accel, gravity=9.81):
    """
    Compute ZMP position in sagittal plane.

    Args:
        com_pos: [x, z] COM position (meters)
        com_vel: [vx, vz] COM velocity (m/s)
        com_accel: [ax, az] COM acceleration (m/s^2)
        gravity: gravitational acceleration (m/s^2)

    Returns:
        zmp_x: ZMP x-position on ground (z=0)
    """
    x_com, z_com = com_pos
    ax_com, az_com = com_accel

    # ZMP formula: x_zmp = x_com - z_com * (ax_com / (az_com + g))
    zmp_x = x_com - z_com * (ax_com / (az_com + gravity))

    return zmp_x

def is_stable(zmp, support_polygon):
    """
    Check if ZMP is inside support polygon.

    Args:
        zmp: [x, y] ZMP position
        support_polygon: list of [x, y] vertices (counterclockwise)

    Returns:
        stable: True if ZMP inside polygon
    """
    # Point-in-polygon test using cross products
    n = len(support_polygon)
    for i in range(n):
        p1 = support_polygon[i]
        p2 = support_polygon[(i + 1) % n]

        # Vector from p1 to p2
        edge = np.array(p2) - np.array(p1)
        # Vector from p1 to ZMP
        to_zmp = np.array(zmp) - np.array(p1)

        # Cross product (2D)
        cross = edge[0] * to_zmp[1] - edge[1] * to_zmp[0]

        if cross < 0:  # ZMP is to the right of edge
            return False

    return True

# Example: Single support phase
com_position = np.array([0.05, 0.85])  # COM slightly ahead of foot, 85cm high
com_velocity = np.array([0.5, 0.0])     # Walking forward at 0.5 m/s
com_acceleration = np.array([0.2, 0.0]) # Accelerating forward

zmp_x = compute_zmp_2d(com_position, com_velocity, com_acceleration)
print(f"ZMP position: {zmp_x:.3f} m")

# Support polygon: single foot (simplified rectangle)
foot_polygon = [
    [-0.05, -0.08],  # Heel left
    [-0.05,  0.12],  # Toe left
    [ 0.05,  0.12],  # Toe right
    [ 0.05, -0.08],  # Heel right
]

stable = is_stable([zmp_x, 0.0], foot_polygon)
print(f"Stable: {stable}")  # False if zmp_x outside foot bounds
```

**Stability Margin**: Distance from ZMP to nearest polygon edge. Larger margins provide more robustness to disturbances.

### Footstep Planning with A* Search

A* searches footstep graph where states are foot placements and edges are kinematic transitions.

**Footstep Planner Implementation**:

```python
import heapq
import math

class FootstepPlanner:
    def __init__(self, obstacle_map, foot_size=(0.1, 0.2)):
        """
        A* footstep planner.

        Args:
            obstacle_map: 2D grid (0=free, 1=occupied)
            foot_size: (width, length) in meters
        """
        self.obstacle_map = obstacle_map
        self.foot_size = foot_size
        self.max_step_length = 0.5  # meters
        self.max_lateral_step = 0.4
        self.max_rotation = 25  # degrees

    def plan(self, start_left, start_right, goal_pos):
        """
        Plan footstep sequence from start to goal.

        Args:
            start_left: (x, y, theta) left foot pose
            start_right: (x, y, theta) right foot pose
            goal_pos: (x, y) goal position

        Returns:
            footsteps: list of (foot, x, y, theta) tuples
        """
        # State: (left_pose, right_pose, last_foot)
        # last_foot: 'L' or 'R' indicating which foot moved last

        start_state = (start_left, start_right, 'R')

        open_set = []
        heapq.heappush(open_set, (0, start_state))

        came_from = {}
        g_score = {start_state: 0}

        def heuristic(state):
            # Euclidean distance from average foot position to goal
            left_pos = state[0][:2]
            right_pos = state[1][:2]
            avg_pos = ((left_pos[0] + right_pos[0]) / 2,
                       (left_pos[1] + right_pos[1]) / 2)
            return math.sqrt((avg_pos[0] - goal_pos[0])**2 +
                           (avg_pos[1] - goal_pos[1])**2)

        def get_successors(state):
            """Generate valid next footsteps."""
            left_pose, right_pose, last_foot = state
            successors = []

            # Next foot to move (alternate)
            if last_foot == 'L':
                moving_foot = 'R'
                stance_pose = left_pose
            else:
                moving_foot = 'L'
                stance_pose = right_pose

            # Sample footstep candidates
            for dx in np.arange(0.2, self.max_step_length, 0.1):
                for dy in np.arange(-self.max_lateral_step,
                                   self.max_lateral_step, 0.1):
                    for dtheta in np.arange(-self.max_rotation,
                                           self.max_rotation, 10):
                        # Compute new foot pose relative to stance foot
                        theta_rad = math.radians(stance_pose[2] + dtheta)
                        new_x = stance_pose[0] + dx * math.cos(theta_rad) - dy * math.sin(theta_rad)
                        new_y = stance_pose[1] + dx * math.sin(theta_rad) + dy * math.cos(theta_rad)
                        new_theta = stance_pose[2] + dtheta

                        new_pose = (new_x, new_y, new_theta)

                        # Check collision
                        if self.is_collision_free(new_pose):
                            if moving_foot == 'L':
                                new_state = (new_pose, right_pose, 'L')
                            else:
                                new_state = (left_pose, new_pose, 'R')
                            successors.append(new_state)

            return successors

        while open_set:
            _, current = heapq.heappop(open_set)

            # Check goal reached
            left_pos = current[0][:2]
            right_pos = current[1][:2]
            avg_pos = ((left_pos[0] + right_pos[0]) / 2,
                       (left_pos[1] + right_pos[1]) / 2)

            if math.sqrt((avg_pos[0] - goal_pos[0])**2 +
                        (avg_pos[1] - goal_pos[1])**2) < 0.2:
                # Reconstruct path
                footsteps = []
                state = current
                while state in came_from:
                    prev_state = came_from[state]
                    # Determine which foot moved
                    if state[0] != prev_state[0]:  # Left foot moved
                        footsteps.append(('L', *state[0]))
                    else:  # Right foot moved
                        footsteps.append(('R', *state[1]))
                    state = prev_state
                return footsteps[::-1]

            for successor in get_successors(current):
                tentative_g = g_score[current] + 1  # Each step costs 1

                if successor not in g_score or tentative_g < g_score[successor]:
                    came_from[successor] = current
                    g_score[successor] = tentative_g
                    f_score = tentative_g + heuristic(successor)
                    heapq.heappush(open_set, (f_score, successor))

        return None  # No path found

    def is_collision_free(self, foot_pose):
        """Check if foot placement collides with obstacles."""
        x, y, theta = foot_pose
        # Simplified: check if foot center is in free space
        # Real implementation would check entire foot polygon
        grid_x = int(x / 0.05)  # 5cm resolution
        grid_y = int(y / 0.05)

        if (0 <= grid_x < self.obstacle_map.shape[1] and
            0 <= grid_y < self.obstacle_map.shape[0]):
            return self.obstacle_map[grid_y, grid_x] == 0
        return False
```

### Whole-Body Control with Task Prioritization

Whole-body controllers solve for joint velocities satisfying multiple tasks with hierarchy.

**Control Architecture**:

```python
import numpy as np
from scipy.optimize import minimize

class WholeBodyController:
    def __init__(self, robot_model, dt=0.01):
        """
        Whole-body controller with task hierarchy.

        Args:
            robot_model: Robot kinematics model
            dt: Control timestep (seconds)
        """
        self.robot = robot_model
        self.dt = dt

    def compute_control(self, current_q, tasks):
        """
        Compute joint velocities satisfying task hierarchy.

        Args:
            current_q: Current joint positions [n_joints]
            tasks: List of (priority, task_func, weight) tuples
                   Higher priority number = higher importance

        Returns:
            q_dot: Joint velocities [n_joints]
        """
        n_joints = len(current_q)

        # Sort tasks by priority (highest first)
        tasks_sorted = sorted(tasks, key=lambda t: t[0], reverse=True)

        # Quadratic program: minimize || J * q_dot - x_dot_desired ||^2
        # Subject to joint limits and higher-priority task constraints

        def objective(q_dot):
            """Minimize tracking error weighted by task priorities."""
            total_error = 0
            for priority, task_func, weight in tasks_sorted:
                J, x_dot_des = task_func(current_q)
                x_dot = J @ q_dot
                error = np.linalg.norm(x_dot - x_dot_des)**2
                total_error += weight * error
            return total_error

        # Constraints from high-priority tasks
        constraints = []
        for priority, task_func, weight in tasks_sorted[:2]:  # Top 2 tasks as hard constraints
            J, x_dot_des = task_func(current_q)
            # Equality constraint: J * q_dot = x_dot_des
            constraints.append({
                'type': 'eq',
                'fun': lambda q_dot, J=J, x_dot_des=x_dot_des: J @ q_dot - x_dot_des
            })

        # Joint velocity limits
        q_dot_max = np.ones(n_joints) * 2.0  # rad/s
        bounds = [(-q_dot_max[i], q_dot_max[i]) for i in range(n_joints)]

        # Solve QP
        result = minimize(objective,
                         x0=np.zeros(n_joints),
                         method='SLSQP',
                         bounds=bounds,
                         constraints=constraints)

        return result.x if result.success else np.zeros(n_joints)

def com_task(robot_model, com_des_velocity):
    """Task: Control COM velocity to maintain ZMP."""
    def task_func(q):
        # Jacobian mapping joint velocities to COM velocity
        J_com = robot_model.compute_com_jacobian(q)
        return J_com, com_des_velocity
    return task_func

def swing_foot_task(robot_model, foot_name, foot_des_velocity):
    """Task: Track swing foot trajectory."""
    def task_func(q):
        # Jacobian for foot end-effector
        J_foot = robot_model.compute_foot_jacobian(q, foot_name)
        return J_foot, foot_des_velocity
    return task_func

def posture_task(robot_model, q_nominal):
    """Task: Maintain nominal posture."""
    def task_func(q):
        # Identity Jacobian (joint space task)
        J = np.eye(len(q))
        q_dot_des = 0.5 * (q_nominal - q)  # Proportional control
        return J, q_dot_des
    return task_func
```

### Push Recovery Strategies

When external forces disturb balance, humanoids must execute recovery steps to prevent falling.

**Capture Point Theory**: The Capture Point (CP) is the ground position where the robot must step to come to rest. If ZMP cannot reach CP within current support polygon, robot will fall unless it takes a step.

**Recovery Algorithm**:

```python
def detect_push_disturbance(com_vel, com_accel, threshold=0.5):
    """
    Detect if robot is being pushed beyond normal walking dynamics.

    Args:
        com_vel: COM velocity [vx, vy] (m/s)
        com_accel: COM acceleration [ax, ay] (m/s^2)
        threshold: Acceleration threshold (m/s^2)

    Returns:
        disturbed: True if push detected
        recovery_direction: Direction to step for recovery
    """
    accel_mag = np.linalg.norm(com_accel)

    if accel_mag > threshold:
        # Push detected, compute recovery direction
        recovery_direction = com_vel / np.linalg.norm(com_vel)
        return True, recovery_direction

    return False, None

def compute_recovery_footstep(com_pos, com_vel, stance_foot_pos):
    """
    Compute recovery footstep using Capture Point.

    Args:
        com_pos: Current COM position [x, y, z]
        com_vel: Current COM velocity [vx, vy, vz]
        stance_foot_pos: Stance foot position [x, y]

    Returns:
        recovery_foot_pos: Where to place recovery foot [x, y]
    """
    g = 9.81
    z_com = com_pos[2]

    # Natural frequency of inverted pendulum
    omega = np.sqrt(g / z_com)

    # Capture Point: where COM velocity will carry it
    cp_x = com_pos[0] + com_vel[0] / omega
    cp_y = com_pos[1] + com_vel[1] / omega

    # Place foot at capture point (with safety margin)
    recovery_foot_pos = np.array([cp_x, cp_y])

    # Limit maximum step length
    step_vector = recovery_foot_pos - stance_foot_pos
    step_length = np.linalg.norm(step_vector)
    max_step = 0.8  # meters

    if step_length > max_step:
        recovery_foot_pos = stance_foot_pos + (step_vector / step_length) * max_step

    return recovery_foot_pos
```

## Seeing It in Action

### Example Scenario: Humanoid Navigating Office with Obstacles

Humanoid walks 8 meters through office, stepping over cables and around chairs.

**Phase 1: Footstep Planning**
- A* footstep planner computes 16-step sequence
- Steps 1-8: straight walking (0.4m step length, 0.3m lateral spacing)
- Steps 9-10: side step to avoid chair (0.15m lateral displacement)
- Steps 11-13: forward steps over cable (0.5m step length, 0.1m height)
- Steps 14-16: final approach to goal

**Phase 2: Walking Execution**
- Whole-body controller executes footsteps
- Step cycle: double support (0.2s) → single support (0.6s) → double support (0.2s)
- During single support: ZMP remains within stance foot (margin greater than 2cm)
- COM height oscillates 0.82-0.88m (6cm vertical motion for dynamic walking)
- Average walking speed: 0.5 m/s

**Phase 3: Push Disturbance**
- External push at t=6.2s (0.3m/s lateral velocity injected)
- Push detection: COM acceleration spikes to 0.8 m/s^2
- Capture Point computed: 0.25m lateral to current stance foot
- Recovery step inserted: right foot steps 0.25m laterally
- Balance restored in 0.9s, walking resumes

**Phase 4: Stair Climbing**
- Robot approaches 3-step staircase (15cm rise, 30cm depth)
- Footstep planner adjusts for vertical stepping
- Step 1: right foot on first stair, ZMP shifts forward
- COM lifts 15cm, hip and knee torques increase 40%
- Steps 2-3: continue climb with increased ground clearance (20cm)
- Total stair time: 4.5 seconds (slower than flat ground)

**Phase 5: Goal Reached**
- Robot arrives at goal position (tolerance 0.3m)
- Final double-support stance for stability
- Total navigation time: 22 seconds for 8 meters (0.36 m/s average)

**Metrics**: 16 footsteps executed, 1 recovery step inserted, 0 falls, ZMP always within support polygon.

## Hands-On Code

### Example: ZMP-Based Walking Controller

```python
"""
ZMP-Based Walking Controller for Humanoid
Demonstrates balance monitoring and footstep execution.
"""

from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.utils.stage import add_reference_to_stage
import numpy as np

class ZMPWalkingController:
    def __init__(self, robot):
        self.robot = robot
        self.com_height = 0.85  # meters
        self.step_length = 0.4
        self.step_width = 0.3
        self.current_foot = 'L'  # Start with left support

    def compute_com_position(self):
        """Compute center of mass from link positions and masses."""
        # Simplified: assume COM at pelvis position
        pelvis_pos = self.robot.get_world_pose()[0]
        return pelvis_pos

    def compute_zmp(self, com_pos, com_accel):
        """Compute ZMP position."""
        g = 9.81
        zmp_x = com_pos[0] - com_pos[2] * (com_accel[0] / (com_accel[2] + g))
        zmp_y = com_pos[1] - com_pos[2] * (com_accel[1] / (com_accel[2] + g))
        return np.array([zmp_x, zmp_y, 0.0])

    def check_stability(self, zmp, support_polygon):
        """Check if ZMP is inside support polygon."""
        # Point-in-polygon test (simplified for rectangle)
        x_min, y_min = np.min(support_polygon, axis=0)[:2]
        x_max, y_max = np.max(support_polygon, axis=0)[:2]

        return (x_min <= zmp[0] <= x_max and y_min <= zmp[1] <= y_max)

    def generate_footstep_plan(self, num_steps):
        """Generate simple footstep sequence."""
        footsteps = []
        x, y = 0.0, 0.0

        for i in range(num_steps):
            if i % 2 == 0:  # Left foot
                y_offset = self.step_width / 2
                foot = 'L'
            else:  # Right foot
                y_offset = -self.step_width / 2
                foot = 'R'

            x += self.step_length
            footsteps.append({
                'foot': foot,
                'position': np.array([x, y_offset, 0.0]),
                'orientation': 0.0  # No rotation
            })

        return footsteps

def main():
    world = World(physics_dt=1.0/60.0)
    world.scene.add_default_ground_plane()

    print("=== ZMP Walking Controller Demo ===\n")

    # Note: This example demonstrates the control logic
    # Actual humanoid robot would require full model with leg joints

    # Simulate walking scenario
    controller = ZMPWalkingController(robot=None)

    print("Generating footstep plan for 8 steps...")
    footsteps = controller.generate_footstep_plan(num_steps=8)

    for i, step in enumerate(footsteps):
        print(f"Step {i+1}: {step['foot']} foot to position {step['position'][:2]}")

    print("\nSimulating ZMP during single support phase...")

    # Simulate COM motion
    com_positions = []
    zmp_positions = []

    for t in np.linspace(0, 0.6, 30):  # 0.6s single support phase
        # COM moves forward with sinusoidal acceleration
        com_x = 0.2 + 0.15 * t
        com_y = 0.15  # Lateral offset during right support
        com_z = 0.85

        # Acceleration (simplified dynamics)
        accel_x = 0.15 * np.cos(2 * np.pi * t)
        accel_y = 0.0
        accel_z = 0.0

        com_pos = np.array([com_x, com_y, com_z])
        com_accel = np.array([accel_x, accel_y, accel_z])

        zmp = controller.compute_zmp(com_pos, com_accel)

        com_positions.append(com_pos)
        zmp_positions.append(zmp)

    # Check stability
    right_foot_polygon = np.array([
        [-0.05, -0.08, 0],
        [-0.05,  0.12, 0],
        [ 0.05,  0.12, 0],
        [ 0.05, -0.08, 0]
    ])

    stable_count = 0
    for zmp in zmp_positions:
        if controller.check_stability(zmp, right_foot_polygon):
            stable_count += 1

    stability_ratio = stable_count / len(zmp_positions)

    print(f"\nStability Analysis:")
    print(f"  - Samples: {len(zmp_positions)}")
    print(f"  - Stable samples: {stable_count}")
    print(f"  - Stability ratio: {stability_ratio*100:.1f}%")

    if stability_ratio > 0.95:
        print("  ✓ Walking gait is stable")
    else:
        print("  ✗ Unstable gait - adjust COM trajectory")

    print("\nController Parameters:")
    print(f"  - COM height: {controller.com_height} m")
    print(f"  - Step length: {controller.step_length} m")
    print(f"  - Step width: {controller.step_width} m")
    print(f"  - Support polygon: {right_foot_polygon[0][:2]} to {right_foot_polygon[2][:2]}")

    simulation_app.close()

if __name__ == "__main__":
    main()
```

## Try It Yourself

### Exercise 1: Implement ZMP Monitoring (Beginner)

**Task**: Create real-time ZMP visualization during simulated walking.

**Requirements**:
1. Load humanoid robot (H1 or similar) in Isaac Sim
2. Execute walking pattern (10 steps forward)
3. Compute ZMP at 60 Hz from:
   - Joint positions, velocities (from robot state)
   - Ground reaction forces (from physics engine)
4. Visualize:
   - Support polygon (convex hull of foot contacts)
   - ZMP trajectory (red dots inside/outside polygon)
   - Safety margin (color-coded: green = safe, yellow = warning, red = unstable)
5. Log stability statistics

**Acceptance Criteria**:
- Compute ZMP at 60 Hz during walk cycle
- Visualize ZMP trajectory overlaid on support polygon
- Detect stability violations (ZMP outside polygon)
- Log percentage of time ZMP is in stable region (target greater than 95%)

**Hints**:
- ZMP from forces: `z ZMP_x = Σ(force_i * position_i.x) / Σ(force_i.z)`
- Support polygon: ConvexHull of foot contact points
- Point-in-polygon test: cross product method (see Chapter 6)
- Visualization: use Isaac Sim's DebugDraw API for real-time overlay
- Typical humanoid walk: 60% single support, 40% double support

**Estimated Time**: 3-4 hours

---

**Solution**: (Simplified - full solution requires Isaac Sim humanoid robot integration)

```python
"""
Exercise 1 Solution: ZMP Monitoring System
Real-time stability visualization during humanoid walking.
"""

import numpy as np
from scipy.spatial import ConvexHull

class ZMPMonitor:
    """Real-time ZMP computation and stability checking."""

    def __init__(self, foot_length=0.2, foot_width=0.1):
        self.foot_length = foot_length
        self.foot_width = foot_width
        self.zmp_history = []
        self.stability_history = []

    def compute_support_polygon(self, left_foot_contacts, right_foot_contacts):
        """
        Compute convex hull of ground contact points.

        Args:
            left_foot_contacts: [(x, y, z), ...] contact points on left foot
            right_foot_contacts: [(x, y, z), ...] contact points on right foot

        Returns:
            polygon: [(x, y), ...] vertices of support polygon (CCW order)
        """
        all_contacts = left_foot_contacts + right_foot_contacts

        if len(all_contacts) < 3:
            # Degenerate case: single foot with 4 corners
            return self._single_foot_polygon(all_contacts[0] if all_contacts else np.zeros(3))

        # Project to ground plane (z=0) and compute convex hull
        points_2d = np.array([[p[0], p[1]] for p in all_contacts])
        hull = ConvexHull(points_2d)
        polygon = points_2d[hull.vertices]

        return polygon

    def _single_foot_polygon(self, foot_center):
        """Generate rectangle for single foot support."""
        x, y = foot_center[0], foot_center[1]
        half_length = self.foot_length / 2
        half_width = self.foot_width / 2

        return np.array([
            [x - half_length, y - half_width],
            [x + half_length, y - half_width],
            [x + half_length, y + half_width],
            [x - half_length, y + half_width]
        ])

    def compute_zmp(self, ground_reaction_forces, contact_positions):
        """
        Compute ZMP from ground reaction forces.

        Args:
            ground_reaction_forces: [(fx, fy, fz), ...] forces at contact points
            contact_positions: [(x, y, z), ...] positions of contact points

        Returns:
            zmp: (x, y) Zero Moment Point on ground plane
        """
        forces = np.array(ground_reaction_forces)
        positions = np.array(contact_positions)

        # Vertical forces only (for simplified 2D ZMP)
        fz = forces[:, 2]
        total_fz = np.sum(fz)

        if total_fz < 1e-6:
            return None  # Robot airborne (no ground contact)

        # Weighted average of contact positions
        zmp_x = np.sum(positions[:, 0] * fz) / total_fz
        zmp_y = np.sum(positions[:, 1] * fz) / total_fz

        return np.array([zmp_x, zmp_y])

    def is_stable(self, zmp, support_polygon):
        """Check if ZMP is inside support polygon."""
        if zmp is None:
            return False

        n = len(support_polygon)
        for i in range(n):
            p1 = support_polygon[i]
            p2 = support_polygon[(i + 1) % n]

            edge = p2 - p1
            to_zmp = zmp - p1

            # Cross product (2D)
            cross = edge[0] * to_zmp[1] - edge[1] * to_zmp[0]

            if cross < 0:  # ZMP outside polygon
                return False

        return True

    def update(self, forces, positions, left_contacts, right_contacts):
        """Update monitoring for current timestep."""
        support_polygon = self.compute_support_polygon(left_contacts, right_contacts)
        zmp = self.compute_zmp(forces, positions)
        stable = self.is_stable(zmp, support_polygon)

        self.zmp_history.append(zmp)
        self.stability_history.append(stable)

        return {
            'zmp': zmp,
            'support_polygon': support_polygon,
            'stable': stable
        }

    def get_statistics(self):
        """Compute stability statistics over monitoring period."""
        stable_count = sum(self.stability_history)
        total_count = len(self.stability_history)

        return {
            'total_samples': total_count,
            'stable_samples': stable_count,
            'stability_ratio': stable_count / total_count if total_count > 0 else 0,
            'pass': (stable_count / total_count) > 0.95 if total_count > 0 else False
        }

# Usage example:
monitor = ZMPMonitor(foot_length=0.2, foot_width=0.1)

# Simulation loop (simplified)
for timestep in range(600):  # 10 seconds at 60 Hz
    # Get robot state from Isaac Sim
    forces = get_ground_reaction_forces()  # From physics engine
    positions = get_contact_positions()  # From collision detection
    left_contacts = get_left_foot_contacts()
    right_contacts = get_right_foot_contacts()

    # Update monitor
    state = monitor.update(forces, positions, left_contacts, right_contacts)

    # Visualize (pseudo-code)
    visualize_support_polygon(state['support_polygon'])
    visualize_zmp(state['zmp'], color='green' if state['stable'] else 'red')

# Final statistics
stats = monitor.get_statistics()
print(f"Stability: {stats['stability_ratio']*100:.1f}% ({stats['stable_samples']}/{stats['total_samples']})")
print(f"Result: {'✓ PASS' if stats['pass'] else '✗ FAIL'}")
```

---

### Exercise 2: A* Footstep Planner for Obstacles (Intermediate)

**Task**: Implement footstep planner navigating around 3 obstacles.

**Requirements**:
1. Environment: 5m x 5m area with 3 circular obstacles (0.5m radius)
2. Footstep graph:
   - Nodes: (x, y, θ, foot) where foot ∈ \{LEFT, RIGHT\}
   - Edges: kinematically valid transitions
3. Constraints:
   - Step length: 0.2 - 0.5m
   - Lateral step: 0.15 - 0.4m
   - Rotation: max 25° per step
4. A* heuristic: Euclidean distance to goal + rotation penalty
5. Collision checking: footprint doesn't intersect obstacles

**Metrics**: Plan computation time, number of footsteps, path smoothness

**Acceptance Criteria**:
- Generate collision-free footstep sequence
- Planning time less than 2 seconds
- Maximum 20 footsteps for 5-meter path
- All footsteps satisfy kinematic constraints (step length, width, rotation)

**Hints**:
- Discretize orientation: 8 directions (45° increments)
- Footprint: rectangle (0.2m x 0.1m)
- Branching factor: ~12 valid next steps per stance
- Priority queue: `heapq` with f-cost = g-cost + h-cost
- Collision check: separating axis theorem for rectangle-circle

**Estimated Time**: 5-6 hours

---

**Solution**: (Outline - full implementation similar to Nav2 A* in Chapter 6, adapted for footsteps)

---

### Exercise 3: Push Recovery Controller (Advanced)

**Task**: Implement Capture Point-based push recovery.

**Requirements**:
1. Detect external pushes using COM acceleration monitoring
2. Compute Capture Point (CP): point where robot must step to stop
3. If CP outside support polygon → execute recovery step
4. Recovery step placement: expand support polygon to contain CP
5. Test with lateral pushes (0.3, 0.5, 0.7 m/s impulse)

**Capture Point Formula**:
```
CP_x = COM_x + COM_vx / ω₀
where ω₀ = sqrt(g / COM_height)
```

**Acceptance Criteria**:
- Detect pushes with less than 100ms latency (detect within 6 frames at 60 Hz)
- Compute recovery footstep within 50ms
- Successfully recover from 0.5 m/s lateral pushes without falling
- Return to nominal walking within 1.5 seconds after push

**Hints**:
- Push detection: `|COM_acceleration| > 2g` threshold
- Capture Point in 2D: `CP = COM_pos + COM_vel / omega_0`
- Recovery step: place foot at CP + safety margin (5cm)
- Stepping reflex: highest priority in whole-body controller
- Test: apply impulse force via `apply_force()` in Isaac Sim

**Estimated Time**: 8-10 hours

---

**Solution**: (Advanced - requires detailed humanoid dynamics integration)

---

## Chapter Summary

This chapter equipped you with the fundamental techniques for humanoid bipedal navigation—the most complex form of robot locomotion, enabling robots to traverse human environments without modification.

**Key Takeaways**:

1. **Zero Moment Point (ZMP)**: The foundation of bipedal stability. ZMP is the ground point where tipping moments from gravity and inertia balance to zero. For stable walking, ZMP must remain inside the support polygon (convex hull of foot contacts) at all times. Violations cause falling.

2. **Support Polygon Dynamics**: Humanoid walking alternates between single support (small polygon = one foot) and double support (larger polygon = both feet). Single support phases are inherently unstable—ZMP must be carefully controlled within the small foot boundary, requiring predictive COM acceleration adjustments.

3. **Footstep Planning**: A* search on footstep graph generates collision-free stepping sequences. Nodes represent valid foot placements (x, y, θ, left/right), edges encode kinematically feasible transitions (step length 0.2-0.5m, lateral width 0.15-0.4m, rotation ≤25°). Heuristic combines Euclidean distance and rotation penalties.

4. **Whole-Body Control**: Humanoids have 20-40 DOF requiring coordinated control. Multi-task optimization solves hierarchical objectives: (1) Balance (ZMP constraint = highest priority), (2) Footstep tracking (swing foot trajectory), (3) Posture (joint limits, comfort). Quadratic programming produces joint velocities satisfying all constraints simultaneously.

5. **Push Recovery**: Capture Point (CP) predicts where robot must step to come to rest. CP = COM_position + COM_velocity / ω₀ where ω₀ = sqrt(g/h). If push disturbs COM causing CP to exit support polygon, reactive stepping places foot at CP location, expanding polygon to regain stability within 200-500ms.

**What We Built**:
- ZMP computation from robot dynamics (Code Example 1)
- Support polygon generation and stability checking (Exercise 1)
- Footstep planner for obstacle avoidance (Exercise 2 outline)
- Push recovery using Capture Point (Exercise 3 outline)

**Comparison: Wheeled vs Legged Navigation**:

| Aspect | Wheeled (Chapter 6) | Humanoid (Chapter 7) |
|--------|---------------------|----------------------|
| Stability | Static (always stable) | Dynamic (must predict balance) |
| DOF | 2-3 (x, y, θ) | 20-40 joints |
| Control Rate | 10-20 Hz | 100-1000 Hz |
| Planning | Path (continuous) | Footsteps (discrete) |
| Terrain | Flat, continuous | Stairs, gaps, uneven |
| Complexity | Medium | Very High |
| Versatility | Indoors, roads | Any human environment |

**Production Deployment Considerations**:
- **Hardware**: Humanoids require high-torque actuators (80-200 Nm per leg joint), high-bandwidth force/torque sensors (1 kHz), and IMUs (1 kHz) for balance
- **Real-Time Requirements**: Balance control at 500-1000 Hz, footstep execution at 100 Hz, planning at 1-10 Hz
- **Safety**: Always maintain ZMP safety margin (3-5cm inside support polygon edges); implement fall detection and safe shutdown
- **Robustness**: Test recovery on 10+ push scenarios (front/back/lateral, various magnitudes); validate on uneven terrain
- **Sim-to-Real**: Model ground compliance, friction variation, actuator delays—bipedal control highly sensitive to parameter mismatch

**Looking Ahead to Chapter 8**:

Now that you can navigate both wheeled robots (Chapter 6) and humanoid robots (Chapter 7), Chapter 8 integrates all modules from Chapters 1-7 into complete AI-powered autonomous systems.

You'll build end-to-end pipelines combining:
- Isaac Sim (Chapters 1-2): Simulation and synthetic data generation
- Perception (Chapter 5): Object detection, pose estimation, semantic segmentation
- SLAM (Chapter 4): Localization and mapping
- Navigation (Chapters 6-7): Path planning and locomotion
- AI Integration (Chapter 8): Reinforcement learning, behavior trees, deployment

The capstone demonstrates warehouse automation: humanoid robot navigates to shelf (Nav2 + SLAM), detects target object (DOPE perception), plans grasp (motion planning), manipulates object while maintaining balance (whole-body control), and delivers to destination. This represents the state-of-the-art in embodied AI—robots that perceive, reason, and act in the physical world.
