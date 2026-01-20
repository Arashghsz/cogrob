#!/usr/bin/env python3
"""
Franka Robot with Gripper Control Example
==========================================

This script demonstrates robot motion combined with gripper control.
Useful for pick-and-place operations.

Before running:
1. Robot is unlocked and FCI is enabled
2. Update the ROBOT_IP with your robot's IP address
"""

from franky import (
    Affine,
    CartesianMotion,
    CartesianWaypointMotion,
    CartesianWaypoint,
    JointMotion,
    Robot,
    ReferenceType,
    Gripper,
    RelativeDynamicsFactor,
)


def main():
    # ==========================================================================
    # CONFIGURATION
    # ==========================================================================
    ROBOT_IP = "192.168.1.100"  # <-- CHANGE THIS TO YOUR ROBOT'S IP
    
    print(f"Connecting to Franka robot at {ROBOT_IP}...")
    
    try:
        # Connect to robot and gripper
        robot = Robot(ROBOT_IP)
        gripper = Gripper(ROBOT_IP)
        print("Connected to robot and gripper!")
        
        # Recover from errors
        robot.recover_from_errors()
        
        # Set slow dynamics for safety
        robot.relative_dynamics_factor = 0.05
        print("Dynamics factor set to 5%")
        
        # ==========================================================================
        # GRIPPER EXAMPLE 1: Open gripper
        # ==========================================================================
        print("\n--- Opening Gripper ---")
        gripper_speed = 0.05  # m/s
        
        input("Press Enter to open gripper...")
        success = gripper.open(gripper_speed)
        print(f"Gripper open: {'Success' if success else 'Failed'}")
        print(f"Current gripper width: {gripper.width * 1000:.1f} mm")
        
        # ==========================================================================
        # GRIPPER EXAMPLE 2: Close gripper to specific width
        # ==========================================================================
        print("\n--- Closing Gripper ---")
        target_width = 0.02  # 20mm
        
        input(f"Press Enter to close gripper to {target_width*1000:.0f}mm...")
        success = gripper.move(target_width, gripper_speed)
        print(f"Gripper move: {'Success' if success else 'Failed'}")
        print(f"Current gripper width: {gripper.width * 1000:.1f} mm")
        
        # ==========================================================================
        # COMBINED MOTION: Multi-waypoint motion
        # ==========================================================================
        print("\n--- Multi-Waypoint Cartesian Motion ---")
        print("This will move the robot through multiple waypoints")
        
        # Create a multi-waypoint motion
        motion = CartesianWaypointMotion([
            # First waypoint: move 5cm in X
            CartesianWaypoint(
                Affine([0.05, 0.0, 0.0]),
                ReferenceType.Relative
            ),
            # Second waypoint: move 5cm in Y (slower)
            CartesianWaypoint(
                Affine([0.0, 0.05, 0.0]),
                ReferenceType.Relative,
                RelativeDynamicsFactor(0.5, 1.0, 1.0)  # 50% slower velocity
            ),
            # Third waypoint: move back
            CartesianWaypoint(
                Affine([-0.05, -0.05, 0.0]),
                ReferenceType.Relative
            ),
        ])
        
        input("Press Enter to execute multi-waypoint motion...")
        robot.move(motion)
        print("Multi-waypoint motion completed!")
        
        # ==========================================================================
        # GRIPPER GRASP EXAMPLE
        # ==========================================================================
        print("\n--- Gripper Grasp Example ---")
        grasp_force = 20.0  # N
        
        input("Press Enter to attempt a grasp (will close until force is detected)...")
        # Grasp with unknown object width
        success = gripper.grasp(
            width=0.0,           # Start from closed
            speed=gripper_speed,
            force=grasp_force,
            epsilon_outer=1.0    # Allow large tolerance
        )
        print(f"Grasp: {'Success' if success else 'No object detected or failed'}")
        print(f"Grasped width: {gripper.width * 1000:.1f} mm")
        
        # Open gripper at the end
        input("\nPress Enter to open gripper and finish...")
        gripper.open(gripper_speed)
        
        print("\n=== Demo completed successfully! ===")
        
    except Exception as e:
        print(f"\nError: {type(e).__name__}: {e}")
        raise


if __name__ == "__main__":
    main()
