"""
Exercise 3 Solution: Simulate Grasping with Contact Physics
Chapter 2: Isaac Sim Basics

Objective: Use Franka arm to grasp dynamic object with 3-stage sequence.
"""

from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.objects import DynamicCuboid
from omni.isaac.core.utils.stage import add_reference_to_stage
from pxr import PhysxSchema, UsdPhysics
import numpy as np

def increase_solver_iterations(world):
    """Increase PhysX solver iterations for better contact stability."""
    stage = world.stage
    scene = PhysxSchema.PhysxSceneAPI.Get(stage, "/physicsScene")

    if not scene:
        # Create physics scene if it doesn't exist
        scene = PhysxSchema.PhysxSceneAPI.Apply(stage.GetPrimAtPath("/physicsScene"))

    # Higher iterations = more stable grasping
    scene.CreateSolverPositionIterationCountAttr(16)  # Default: 4
    scene.CreateSolverVelocityIterationCountAttr(8)   # Default: 1

    print("✓ Increased solver iterations for grasp stability")

def create_target_object(world):
    """Create small cube for grasping."""

    target = world.scene.add(
        DynamicCuboid(
            prim_path="/World/TargetCube",
            name="target_cube",
            position=np.array([0.5, 0.0, 0.3]),  # Within arm's workspace
            size=np.array([0.05, 0.05, 0.05]),   # 5cm cube
            color=np.array([0.2, 0.6, 0.9]),     # Blue
            mass=0.2  # 200g (light object)
        )
    )

    print(f"✓ Created target cube at (0.5, 0.0, 0.3)")
    return target

def get_pre_grasp_pose():
    """
    Joint configuration for pre-grasp (above object, gripper open).

    Returns: Joint positions [9 DOF]
    """
    # Arm configuration reaching toward (0.5, 0.0, 0.4) - above object
    # These values are manually tuned for the target position
    return np.array([
        0.0,      # joint1: shoulder pan
        -0.5,     # joint2: shoulder lift
        0.0,      # joint3: elbow pan
        -2.0,     # joint4: elbow lift
        0.0,      # joint5: wrist pan
        1.5,      # joint6: wrist lift
        0.785,    # joint7: wrist roll (45°)
        0.04,     # finger1: open (4cm)
        0.04      # finger2: open (4cm)
    ])

def get_grasp_pose():
    """
    Joint configuration for grasping (lowered, gripper closing).

    Returns: Joint positions [9 DOF]
    """
    # Similar to pre-grasp but lower wrist and close gripper
    return np.array([
        0.0,
        -0.5,
        0.0,
        -2.2,     # Lower elbow slightly
        0.0,
        1.3,      # Lower wrist to object height
        0.785,
        0.01,     # finger1: closing
        0.01      # finger2: closing
    ])

def get_lift_pose():
    """
    Joint configuration for lifting (raise wrist, gripper closed).

    Returns: Joint positions [9 DOF]
    """
    return np.array([
        0.0,
        -0.5,
        0.0,
        -2.0,
        0.0,
        1.7,      # Lift wrist 30cm higher
        0.785,
        0.01,     # finger1: closed
        0.01      # finger2: closed
    ])

def execute_stage(franka, target_positions, duration_steps, world, stage_name):
    """
    Execute one stage of grasp sequence.

    Args:
        franka: Articulation object
        target_positions: Target joint positions
        duration_steps: Number of simulation steps
        world: World object
        stage_name: Name for logging

    Returns:
        Final object position
    """
    print(f"\n--- {stage_name} ---")

    for i in range(duration_steps):
        franka.set_joint_positions(target_positions)
        world.step(render=True)

        if i % 30 == 0:  # Every 0.5s
            current_positions = franka.get_joint_positions()
            error = np.linalg.norm(current_positions - target_positions)
            print(f"  Time {i/60.0:.1f}s: Position error = {error:.4f}")

    # Get final arm and object states
    arm_positions = franka.get_joint_positions()
    print(f"✓ {stage_name} complete")

    return arm_positions

def check_grasp_success(target_object, initial_height, threshold=0.55):
    """
    Verify successful grasp by checking object height.

    Args:
        target_object: The grasped object
        initial_height: Initial Z position
        threshold: Minimum height for success (m)

    Returns:
        success (bool), final_height (float)
    """
    position, _ = target_object.get_world_pose()
    final_height = position[2]

    height_increase = final_height - initial_height

    print(f"\n=== Grasp Validation ===")
    print(f"Initial height: {initial_height:.3f}m")
    print(f"Final height:   {final_height:.3f}m")
    print(f"Height increase: {height_increase:.3f}m")

    if final_height > threshold:
        return True, final_height
    else:
        return False, final_height

def check_object_stability(target_object, world, num_checks=30):
    """
    Check if object velocity is stable (grasped firmly).

    Args:
        target_object: The grasped object
        world: World object
        num_checks: Number of steps to check

    Returns:
        is_stable (bool)
    """
    velocities = []

    for _ in range(num_checks):
        # Get linear velocity
        vel = target_object.get_linear_velocity()
        speed = np.linalg.norm(vel)
        velocities.append(speed)
        world.step(render=False)  # Don't render during check

    avg_velocity = np.mean(velocities)
    max_velocity = np.max(velocities)

    print(f"\nStability check:")
    print(f"  Average velocity: {avg_velocity:.4f} m/s")
    print(f"  Max velocity:     {max_velocity:.4f} m/s")

    # Object is stable if moving very slowly
    is_stable = avg_velocity < 0.05 and max_velocity < 0.1

    return is_stable

def main():
    # Create world
    world = World(physics_dt=1.0/60.0, rendering_dt=1.0/60.0)
    world.scene.add_default_ground_plane()

    print("=== Exercise 3: Grasping Simulation ===\n")

    # Spawn Franka arm
    franka_prim_path = "/World/Franka"
    franka_usd_path = "omniverse://localhost/NVIDIA/Assets/Isaac/2023.1.0/Isaac/Robots/Franka/franka.usd"

    add_reference_to_stage(usd_path=franka_usd_path, prim_path=franka_prim_path)

    franka = world.scene.add(
        Articulation(prim_path=franka_prim_path, name="franka_arm")
    )

    # Create target object
    target_cube = create_target_object(world)

    # Increase physics solver quality
    increase_solver_iterations(world)

    # Initialize
    world.reset()

    # Record initial object position
    initial_pos, _ = target_cube.get_world_pose()
    initial_height = initial_pos[2]
    print(f"✓ Initial object height: {initial_height:.3f}m\n")

    # === STAGE 1: Pre-Grasp ===
    pre_grasp = get_pre_grasp_pose()
    execute_stage(franka, pre_grasp, 180, world, "STAGE 1: Pre-Grasp (Approach)")

    # === STAGE 2: Grasp (Lower and Close) ===
    grasp = get_grasp_pose()
    execute_stage(franka, grasp, 120, world, "STAGE 2: Grasp (Lower & Close)")

    # Check contact
    print("\nChecking gripper contact...")
    # Give physics time to establish contact
    for _ in range(30):
        world.step(render=True)

    # === STAGE 3: Lift ===
    lift = get_lift_pose()
    execute_stage(franka, lift, 180, world, "STAGE 3: Lift Object")

    # Allow object to settle
    print("\nAllowing object to settle...")
    for _ in range(60):
        world.step(render=True)

    # === Validation ===

    # Check 1: Height test
    success, final_height = check_grasp_success(target_cube, initial_height, threshold=0.55)

    # Check 2: Stability test
    is_stable = check_object_stability(target_cube, world, num_checks=30)

    # === Results ===
    print("\n" + "="*50)
    print("=== GRASP SIMULATION RESULTS ===")
    print("="*50)

    if success and is_stable:
        print("✓ GRASP SUCCESSFUL!")
        print(f"  • Object lifted to {final_height:.3f}m (threshold: 0.55m)")
        print("  • Object stable (firmly grasped)")
    elif success and not is_stable:
        print("⚠️  PARTIAL SUCCESS")
        print(f"  • Object lifted to {final_height:.3f}m")
        print("  • Object unstable (may slip)")
    else:
        print("❌ GRASP FAILED")
        print(f"  • Object only at {final_height:.3f}m (threshold: 0.55m)")
        if is_stable:
            print("  • Object stable but not lifted")
        else:
            print("  • Object dropped or slipped")

    print("\n" + "="*50)
    print("\nSimulation complete. Close window to exit.")

    # Keep simulation running for inspection
    while True:
        world.step(render=True)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nExiting...")
        simulation_app.close()
