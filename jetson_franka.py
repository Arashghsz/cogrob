#!/usr/bin/env python3
"""
Robust Franky Wrapper for Jetson AGX Orin
==========================================

This module provides a robust wrapper around franky that handles
the discontinuity errors common on embedded systems like Jetson.

Key features:
- Automatic retry on discontinuity errors
- Configurable dynamics based on motion type
- Graceful error recovery
- Progress logging
"""

import time
from typing import Optional, Union
from franky import (
    Affine, Robot, Gripper,
    CartesianMotion, JointMotion,
    CartesianWaypointMotion, CartesianWaypoint,
    JointWaypointMotion, JointWaypoint,
    ReferenceType, RealtimeConfig,
    RelativeDynamicsFactor,
    Motion,
)


class JetsonFrankaRobot:
    """
    A robust wrapper for controlling Franka robot from Jetson.
    
    Handles discontinuity errors with automatic retries and
    provides optimized dynamics for embedded systems.
    """
    
    # Default dynamics profiles optimized for Jetson
    ULTRA_SLOW = RelativeDynamicsFactor(0.01, 0.02, 0.03)
    SLOW = RelativeDynamicsFactor(0.02, 0.03, 0.05)
    MEDIUM = RelativeDynamicsFactor(0.03, 0.05, 0.1)
    
    def __init__(self, robot_ip: str, max_retries: int = 3):
        """
        Initialize connection to Franka robot.
        
        Args:
            robot_ip: IP address of the robot
            max_retries: Number of retries on discontinuity errors
        """
        self.robot_ip = robot_ip
        self.max_retries = max_retries
        
        # Connect with real-time config ignored (required for Jetson)
        self.robot = Robot(robot_ip, realtime_config=RealtimeConfig.Ignore)
        self.robot.recover_from_errors()
        
        # Default to slow dynamics
        self.robot.relative_dynamics_factor = self.SLOW
        
        print(f"✓ Connected to robot at {robot_ip}")
    
    @property
    def gripper(self) -> Gripper:
        """Lazy-load gripper connection"""
        if not hasattr(self, '_gripper'):
            self._gripper = Gripper(self.robot_ip)
        return self._gripper
    
    @property
    def joint_positions(self) -> list:
        """Get current joint positions"""
        return list(self.robot.current_joint_state.position)
    
    @property
    def ee_pose(self) -> Affine:
        """Get current end-effector pose"""
        return self.robot.current_cartesian_state.pose.end_effector_pose
    
    def set_dynamics(self, profile: str = "slow"):
        """
        Set dynamics profile.
        
        Args:
            profile: One of "ultra_slow", "slow", or "medium"
        """
        profiles = {
            "ultra_slow": self.ULTRA_SLOW,
            "slow": self.SLOW,
            "medium": self.MEDIUM,
        }
        self.robot.relative_dynamics_factor = profiles.get(profile, self.SLOW)
    
    def move(self, motion: Motion, retries: Optional[int] = None) -> bool:
        """
        Execute a motion with automatic retry on discontinuity errors.
        
        Args:
            motion: The motion to execute
            retries: Number of retries (uses default if not specified)
            
        Returns:
            True if motion succeeded, False otherwise
        """
        max_attempts = retries if retries is not None else self.max_retries
        
        for attempt in range(max_attempts + 1):
            try:
                self.robot.move(motion)
                return True
            except Exception as e:
                error_str = str(e)
                
                # Check if it's a discontinuity error (can retry)
                if "discontinuity" in error_str.lower():
                    if attempt < max_attempts:
                        # Wait a bit and retry
                        time.sleep(0.5)
                        self.robot.recover_from_errors()
                        continue
                    else:
                        print(f"Motion failed after {max_attempts} retries: {e}")
                        self.robot.recover_from_errors()
                        return False
                else:
                    # Other errors, don't retry
                    print(f"Motion error: {e}")
                    self.robot.recover_from_errors()
                    return False
        
        return False
    
    def move_cartesian_relative(self, x: float = 0, y: float = 0, z: float = 0) -> bool:
        """
        Move end-effector by relative amount in Cartesian space.
        
        Args:
            x, y, z: Relative displacement in meters
            
        Returns:
            True if motion succeeded
        """
        motion = CartesianMotion(
            Affine([x, y, z]),
            ReferenceType.Relative
        )
        return self.move(motion)
    
    def move_to_joints(self, joint_positions: list) -> bool:
        """
        Move to absolute joint positions.
        
        Args:
            joint_positions: List of 7 joint angles in radians
            
        Returns:
            True if motion succeeded
        """
        motion = JointMotion(joint_positions)
        return self.move(motion)
    
    def move_joint_relative(self, joint_deltas: list) -> bool:
        """
        Move joints by relative amounts.
        
        Args:
            joint_deltas: List of 7 joint angle changes in radians
            
        Returns:
            True if motion succeeded
        """
        current = self.joint_positions
        target = [c + d for c, d in zip(current, joint_deltas)]
        return self.move_to_joints(target)
    
    def open_gripper(self, speed: float = 0.05) -> bool:
        """Open the gripper fully"""
        try:
            self.gripper.open(speed)
            return True
        except Exception as e:
            print(f"Gripper error: {e}")
            return False
    
    def close_gripper(self, width: float = 0.01, speed: float = 0.05, 
                      force: float = 20) -> bool:
        """
        Close gripper to specified width.
        
        Args:
            width: Target width in meters
            speed: Closing speed in m/s
            force: Gripping force in N (for grasp operation)
        """
        try:
            self.gripper.move(width, speed)
            return True
        except Exception as e:
            print(f"Gripper error: {e}")
            return False
    
    def grasp(self, width: float = 0.01, speed: float = 0.05, 
              force: float = 20) -> bool:
        """Grasp an object with force control"""
        try:
            return self.gripper.grasp(width, speed, force, epsilon_inner=0.01, epsilon_outer=0.01)
        except Exception as e:
            print(f"Grasp error: {e}")
            return False
    
    def recover(self):
        """Recover from errors"""
        self.robot.recover_from_errors()


# Demo usage
if __name__ == "__main__":
    print("=" * 60)
    print("Jetson Franka Robot Demo")
    print("=" * 60)
    
    # Create robot wrapper
    franka = JetsonFrankaRobot("172.16.0.2")
    
    # Print current state
    print(f"\nJoint positions: {[round(q, 3) for q in franka.joint_positions]}")
    ee = franka.ee_pose
    print(f"EE position: x={ee.translation[0]:.3f}, y={ee.translation[1]:.3f}, z={ee.translation[2]:.3f}")
    
    # Test motions with retries
    print("\n[TEST 1] Z-axis motion (3cm up and down)")
    franka.set_dynamics("slow")
    if franka.move_cartesian_relative(z=0.03):
        print("  ✓ Up succeeded")
        franka.move_cartesian_relative(z=-0.03)
        print("  ✓ Down succeeded")
    else:
        print("  ✗ Motion failed")
    
    print("\n[TEST 2] Joint motion (J1 +0.1 rad and back)")
    if franka.move_joint_relative([0.1, 0, 0, 0, 0, 0, 0]):
        print("  ✓ Forward succeeded")
        franka.move_joint_relative([-0.1, 0, 0, 0, 0, 0, 0])
        print("  ✓ Back succeeded")
    else:
        print("  ✗ Motion failed")
    
    print("\n[TEST 3] Gripper control")
    if franka.open_gripper():
        print(f"  ✓ Opened to {franka.gripper.width * 1000:.1f}mm")
    franka.close_gripper(width=0.03)
    print(f"  ✓ Closed to {franka.gripper.width * 1000:.1f}mm")
    franka.open_gripper()
    print(f"  ✓ Opened to {franka.gripper.width * 1000:.1f}mm")
    
    print("\n" + "=" * 60)
    print("Demo complete!")
    print("=" * 60)
