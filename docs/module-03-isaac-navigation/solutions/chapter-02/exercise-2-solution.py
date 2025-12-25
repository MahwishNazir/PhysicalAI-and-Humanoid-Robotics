"""
Exercise 2 Solution: Build Maze and Navigate Mobile Robot
Chapter 2: Isaac Sim Basics

Objective: Create maze environment, implement waypoint navigation with heading control.
"""

from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.objects import FixedCuboid, DynamicCuboid
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.rotations import quat_to_euler_angles
import numpy as np

def create_maze(world):
    """Create a 10m x 10m maze with walls."""

    walls = []
    wall_specs = [
        # [x, y, length, orientation]
        # Horizontal walls (oriented along X-axis)
        ([2.0, 1.0, 0.5], [4.0, 0.2, 1.0]),   # Wall 1
        ([2.0, -1.0, 0.5], [4.0, 0.2, 1.0]),  # Wall 2
        ([5.0, 2.0, 0.5], [3.0, 0.2, 1.0]),   # Wall 3
        ([5.0, -2.0, 0.5], [3.0, 0.2, 1.0]),  # Wall 4

        # Vertical walls (oriented along Y-axis)
        ([4.0, 0.0, 0.5], [0.2, 2.0, 1.0]),   # Wall 5 (center barrier)
        ([7.0, 1.0, 0.5], [0.2, 2.0, 1.0]),   # Wall 6
        ([7.0, -1.0, 0.5], [0.2, 2.0, 1.0]),  # Wall 7

        # Outer boundary walls
        ([-1.0, 0.0, 0.5], [0.2, 10.0, 1.0]), # Left wall
        ([10.0, 0.0, 0.5], [0.2, 10.0, 1.0]), # Right wall
    ]

    for i, (position, size) in enumerate(wall_specs):
        wall = world.scene.add(
            FixedCuboid(
                prim_path=f"/World/Wall_{i}",
                name=f"wall_{i}",
                position=np.array(position),
                size=np.array(size),
                color=np.array([0.5, 0.5, 0.5])  # Gray
            )
        )
        walls.append(wall)

    print(f"✓ Created maze with {len(walls)} walls")
    return walls

def create_goal_marker(world):
    """Create goal marker at maze exit."""

    goal = world.scene.add(
        DynamicCuboid(
            prim_path="/World/GoalMarker",
            name="goal_marker",
            position=np.array([9.0, 0.0, 0.25]),
            size=np.array([0.5, 0.5, 0.5]),
            color=np.array([1.0, 0.0, 0.0]),  # Red (not reached)
            mass=1.0
        )
    )

    print("✓ Created goal marker at (9.0, 0.0)")
    return goal

def extract_yaw_from_quaternion(quaternion):
    """Extract yaw angle (rotation around Z-axis) from quaternion."""
    # quaternion format: [x, y, z, w]
    euler = quat_to_euler_angles(quaternion)  # Returns [roll, pitch, yaw]
    yaw = euler[2]
    return yaw

def compute_steering_control(current_pos, current_yaw, target_pos, base_speed=4.0, k_turn=2.0):
    """
    Compute differential steering commands.

    Args:
        current_pos: [x, y] current position
        current_yaw: current heading angle (radians)
        target_pos: [x, y] target waypoint
        base_speed: forward velocity (rad/s)
        k_turn: turning gain

    Returns:
        [left_wheel_velocity, right_wheel_velocity]
    """
    # Vector to target
    to_target = target_pos - current_pos
    distance = np.linalg.norm(to_target)

    if distance < 0.1:  # Very close
        return np.array([0.0, 0.0])

    # Desired heading
    target_yaw = np.arctan2(to_target[1], to_target[0])

    # Heading error (wrapped to [-pi, pi])
    yaw_error = target_yaw - current_yaw
    yaw_error = np.arctan2(np.sin(yaw_error), np.cos(yaw_error))

    # Differential steering: turn_rate adjusts left/right wheel speeds
    turn_rate = k_turn * yaw_error

    left_velocity = base_speed - turn_rate
    right_velocity = base_speed + turn_rate

    return np.array([left_velocity, right_velocity])

def navigate_maze(carter, goal_marker, world):
    """Navigate through waypoints to maze exit."""

    # Waypoints defining path through maze
    waypoints = [
        np.array([0.0, 0.0]),   # Start
        np.array([1.5, 0.0]),   # Before first barriers
        np.array([3.0, 0.0]),   # Between first walls
        np.array([5.5, 1.5]),   # Turn up and right
        np.array([8.0, 1.5]),   # Along top corridor
        np.array([9.0, 0.0]),   # Exit (goal)
    ]

    current_waypoint_idx = 0
    goal_reached = False

    max_steps = 1800  # 30 seconds at 60 Hz

    print("\n=== Starting Maze Navigation ===\n")

    for step in range(max_steps):
        # Get robot state
        position, orientation = carter.get_world_pose()
        current_pos = np.array([position[0], position[1]])
        current_yaw = extract_yaw_from_quaternion(orientation)

        # Current target waypoint
        if current_waypoint_idx < len(waypoints):
            target_pos = waypoints[current_waypoint_idx]

            # Distance to current waypoint
            distance = np.linalg.norm(target_pos - current_pos)

            # Check if waypoint reached
            if distance < 0.3:  # Within 30cm
                current_waypoint_idx += 1
                if current_waypoint_idx < len(waypoints):
                    print(f"✓ Waypoint {current_waypoint_idx}/{len(waypoints)} reached")
                else:
                    print("✓ All waypoints reached!")

            # Compute steering control
            velocities = compute_steering_control(
                current_pos, current_yaw, target_pos,
                base_speed=4.0, k_turn=3.0
            )
        else:
            # Reached all waypoints, stop
            velocities = np.array([0.0, 0.0])

            # Check if at goal marker (within 0.5m)
            goal_pos, _ = goal_marker.get_world_pose()
            goal_distance = np.linalg.norm(current_pos - np.array([goal_pos[0], goal_pos[1]]))

            if goal_distance < 0.5 and not goal_reached:
                print(f"\n🎯 GOAL REACHED! Distance: {goal_distance:.2f}m")
                # Change goal marker to green
                goal_marker.set_default_state(color=np.array([0.0, 1.0, 0.0]))
                goal_marker.set_color(np.array([0.0, 1.0, 0.0]))
                goal_reached = True
                break

        # Apply control
        carter.set_joint_velocity_targets(velocities)

        # Step simulation
        world.step(render=True)

        # Print status every 60 steps (1 second)
        if step % 60 == 0:
            print(f"Time {step/60.0:.1f}s: Pos=[{current_pos[0]:.2f}, {current_pos[1]:.2f}], "
                  f"Waypoint {current_waypoint_idx}/{len(waypoints)}")

    return goal_reached

def main():
    # Create world
    world = World(physics_dt=1.0/60.0, rendering_dt=1.0/60.0)
    world.scene.add_default_ground_plane()

    print("=== Exercise 2: Maze Navigation ===\n")

    # Create maze
    walls = create_maze(world)

    # Create goal marker
    goal_marker = create_goal_marker(world)

    # Spawn Carter robot at entrance
    carter_prim_path = "/World/Carter"
    carter_usd_path = "omniverse://localhost/NVIDIA/Assets/Isaac/2023.1.0/Isaac/Robots/Carter/carter_v1.usd"

    add_reference_to_stage(usd_path=carter_usd_path, prim_path=carter_prim_path)

    carter = world.scene.add(
        Articulation(prim_path=carter_prim_path, name="carter_robot")
    )
    carter.set_world_pose(position=np.array([0.0, 0.0, 0.1]))

    # Initialize
    world.reset()
    print("✓ Maze initialized\n")

    # Navigate
    success = navigate_maze(carter, goal_marker, world)

    # Results
    print("\n=== Maze Navigation Results ===")
    if success:
        print("✓ SUCCESS: Robot reached goal marker!")
        print("✓ Goal marker changed to GREEN")
    else:
        print("⚠️  Navigation incomplete (timeout)")

    print("\nSimulation complete. Close window to exit.")

    # Keep simulation running
    while True:
        world.step(render=True)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nExiting...")
        simulation_app.close()
