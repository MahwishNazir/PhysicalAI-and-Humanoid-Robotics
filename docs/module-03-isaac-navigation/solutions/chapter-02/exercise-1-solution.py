"""
Exercise 1 Solution: Spawn Franka Arm and Control Joints
Chapter 2: Isaac Sim Basics

Objective: Load Franka Panda arm, inspect joints, move to home configuration.
"""

from omni.isaac.kit import SimulationApp

# Launch Isaac Sim
simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.utils.stage import add_reference_to_stage
import numpy as np

def main():
    # Create world
    world = World(physics_dt=1.0/60.0, rendering_dt=1.0/60.0)
    world.scene.add_default_ground_plane()

    print("=== Exercise 1: Franka Arm Control ===\n")

    # Spawn Franka Panda arm
    franka_prim_path = "/World/Franka"
    franka_usd_path = "omniverse://localhost/NVIDIA/Assets/Isaac/2023.1.0/Isaac/Robots/Franka/franka.usd"

    print("Spawning Franka Panda arm...")
    add_reference_to_stage(usd_path=franka_usd_path, prim_path=franka_prim_path)

    # Create articulation interface
    franka = world.scene.add(
        Articulation(
            prim_path=franka_prim_path,
            name="franka_arm"
        )
    )

    # Initialize physics
    world.reset()

    # Inspect joints
    num_dof = franka.num_dof
    dof_names = franka.dof_names

    print(f"\n✓ Franka has {num_dof} DOFs")
    print(f"✓ Joint names: {dof_names}\n")

    # Expected: 9 DOFs
    # - 7 arm joints (panda_joint1 through panda_joint7)
    # - 2 gripper fingers (panda_finger_joint1, panda_finger_joint2)

    if num_dof != 9:
        print(f"⚠️  Warning: Expected 9 DOFs, got {num_dof}")

    # Home configuration
    # Arm joints: [0.0, -45°, 0.0, -135°, 0.0, 90°, 45°]
    # Gripper: [0.04, 0.04] (partially open, in meters)
    home_positions = np.array([
        0.0,      # joint1: shoulder pan
        -0.785,   # joint2: shoulder lift (-45°)
        0.0,      # joint3: elbow pan
        -2.356,   # joint4: elbow lift (-135°)
        0.0,      # joint5: wrist pan
        1.571,    # joint6: wrist lift (90°)
        0.785,    # joint7: wrist roll (45°)
        0.04,     # finger1 (4cm open)
        0.04      # finger2 (4cm open)
    ])

    print("Moving to home configuration...")
    print(f"Target positions: {home_positions}\n")

    # Run simulation for 5 seconds
    steps = 300  # 5 seconds at 60 Hz

    for i in range(steps):
        # Command home position (PD controller will drive joints)
        franka.set_joint_positions(home_positions)

        # Step simulation
        world.step(render=True)

        # Print progress every second
        if i % 60 == 0:
            current_positions = franka.get_joint_positions()
            position_error = np.linalg.norm(current_positions - home_positions)
            print(f"Time {i/60.0:.1f}s: Position error = {position_error:.4f} rad")

    # Final check
    final_positions = franka.get_joint_positions()
    final_error = np.linalg.norm(final_positions - home_positions)

    print(f"\n=== Final Results ===")
    print(f"Target positions:  {home_positions}")
    print(f"Actual positions:  {final_positions}")
    print(f"Position error:    {final_error:.6f} rad")

    if final_error < 0.01:  # Within 0.01 radians (~0.57 degrees)
        print("\n✓ SUCCESS: Arm reached home configuration!")
    else:
        print(f"\n⚠️  Warning: Position error {final_error:.4f} exceeds threshold 0.01")

    print("\nSimulation complete. Press Ctrl+C to exit.")

    # Keep simulation running for inspection
    while True:
        world.step(render=True)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nExiting...")
        simulation_app.close()
