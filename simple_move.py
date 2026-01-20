#!/usr/bin/env python3
"""
Simple Franka Robot Motion Example using Franky
================================================

This script demonstrates basic robot control using the franky library.
Before running this script:
1. Make sure the robot is unlocked (blue button) in the Franka web interface
2. Enable FCI (Franka Control Interface) in the web interface
3. Replace the IP address with your robot's actual IP address

IMPORTANT: Start with very slow movements! The relative_dynamics_factor is set 
to 0.05 (5% of max speed) for safety.
"""

from franky import (
    Affine,
    CartesianMotion,
    JointMotion,
    Robot,
    ReferenceType,
    Gripper,
)


def main():
    # ==========================================================================
    # CONFIGURATION - Update this with your robot's IP address!
    # ==========================================================================
    ROBOT_IP = "192.168.1.100"  # <-- CHANGE THIS TO YOUR ROBOT'S IP
    
    print(f"Connecting to Franka robot at {ROBOT_IP}...")
    
    try:
        # Connect to the robot
        robot = Robot(ROBOT_IP)
        print("Connected successfully!")
        
        # Recover from any previous errors
        robot.recover_from_errors()
        print("Recovered from any previous errors")
        
        # IMPORTANT: Set a very slow dynamics factor for safety
        # This limits velocity, acceleration, and jerk to 5% of maximum
        robot.relative_dynamics_factor = 0.05
        print("Set relative dynamics factor to 0.05 (5% of max speed)")
        
        # Get and print current robot state
        print("\n--- Current Robot State ---")
        joint_state = robot.current_joint_state
        print(f"Joint positions: {[round(p, 3) for p in joint_state.position]}")
        
        cartesian_state = robot.current_cartesian_state
        ee_pose = cartesian_state.pose.end_effector_pose
        print(f"End-effector position: x={ee_pose.translation[0]:.3f}, "
              f"y={ee_pose.translation[1]:.3f}, z={ee_pose.translation[2]:.3f}")
        
        # ==========================================================================
        # MOTION EXAMPLE 1: Small relative Cartesian motion
        # ==========================================================================
        print("\n--- Motion Example 1: Small Cartesian Motion ---")
        print("Moving 5cm in positive X direction relative to current position...")
        
        # Create a relative Cartesian motion (move 5cm in X direction)
        motion1 = CartesianMotion(
            Affine([0.05, 0.0, 0.0]),  # 5cm in X direction
            ReferenceType.Relative     # Relative to current position
        )
        
        input("Press Enter to execute motion (or Ctrl+C to abort)...")
        robot.move(motion1)
        print("Motion 1 completed!")
        
        # ==========================================================================
        # MOTION EXAMPLE 2: Move back to original position
        # ==========================================================================
        print("\n--- Motion Example 2: Move Back ---")
        print("Moving -5cm in X direction (back to start)...")
        
        motion2 = CartesianMotion(
            Affine([-0.05, 0.0, 0.0]),  # -5cm in X direction
            ReferenceType.Relative
        )
        
        input("Press Enter to execute motion (or Ctrl+C to abort)...")
        robot.move(motion2)
        print("Motion 2 completed!")
        
        # ==========================================================================
        # MOTION EXAMPLE 3: Small vertical motion
        # ==========================================================================
        print("\n--- Motion Example 3: Small Vertical Motion ---")
        print("Moving 3cm up in Z direction...")
        
        motion3 = CartesianMotion(
            Affine([0.0, 0.0, 0.03]),  # 3cm up in Z direction
            ReferenceType.Relative
        )
        
        input("Press Enter to execute motion (or Ctrl+C to abort)...")
        robot.move(motion3)
        print("Motion 3 completed!")
        
        # Move back down
        print("Moving 3cm down...")
        motion4 = CartesianMotion(
            Affine([0.0, 0.0, -0.03]),  # 3cm down
            ReferenceType.Relative
        )
        
        input("Press Enter to execute motion (or Ctrl+C to abort)...")
        robot.move(motion4)
        print("Motion 4 completed!")
        
        print("\n=== All motions completed successfully! ===")
        
    except Exception as e:
        print(f"\nError: {type(e).__name__}: {e}")
        print("\nTroubleshooting tips:")
        print("1. Check that the robot IP address is correct")
        print("2. Make sure FCI is enabled in the web interface")
        print("3. Ensure the robot brakes are unlocked")
        print("4. Check if there are any error states in the web interface")
        raise


if __name__ == "__main__":
    main()
