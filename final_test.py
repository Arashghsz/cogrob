#!/usr/bin/env python3
"""
Final Optimized Franky Configuration for Jetson AGX Orin
=========================================================

Based on comprehensive testing, these are the WORKING configurations:

For Joint Motions:
  robot.relative_dynamics_factor = RelativeDynamicsFactor(0.03, 0.05, 0.1)
  
For Cartesian Motions (more conservative):
  robot.relative_dynamics_factor = RelativeDynamicsFactor(0.02, 0.03, 0.05)

Key Requirements:
1. Close Chrome/browser before running
2. Run: sudo jetson_clocks  
3. Set CPU governors to performance mode
4. Keep system load low (< 6 for 12 cores)
"""

import time
from franky import (
    Affine, Robot, Gripper,
    CartesianMotion, JointMotion,
    CartesianWaypointMotion, CartesianWaypoint,
    JointWaypointMotion, JointWaypoint,
    ReferenceType, RealtimeConfig,
    RelativeDynamicsFactor,
)

ROBOT_IP = "172.16.0.2"

# Dynamics profiles
JOINT_DYNAMICS = RelativeDynamicsFactor(0.03, 0.05, 0.1)
CARTESIAN_DYNAMICS = RelativeDynamicsFactor(0.02, 0.03, 0.05)  # More conservative


def main():
    print("=" * 60)
    print("FINAL FRANKY TEST FOR JETSON")
    print("=" * 60)
    
    robot = Robot(ROBOT_IP, realtime_config=RealtimeConfig.Ignore)
    robot.recover_from_errors()
    
    initial_q = list(robot.current_joint_state.position)
    ee = robot.current_cartesian_state.pose.end_effector_pose
    print(f"Initial EE: x={ee.translation[0]:.3f}, y={ee.translation[1]:.3f}, z={ee.translation[2]:.3f}")
    
    all_passed = True
    
    # Test 1: Joint motion (should work reliably)
    print("\n[TEST 1] Joint Motion")
    robot.relative_dynamics_factor = JOINT_DYNAMICS
    try:
        target_q = initial_q.copy()
        target_q[0] += 0.1
        robot.move(JointMotion(target_q))
        robot.move(JointMotion(initial_q))
        print("  ✓ PASS: Joint motion")
    except Exception as e:
        print(f"  ✗ FAIL: {e}")
        robot.recover_from_errors()
        all_passed = False
    
    time.sleep(0.5)
    
    # Test 2: Z-axis Cartesian (should work reliably)
    print("\n[TEST 2] Z-axis Cartesian Motion")
    robot.relative_dynamics_factor = CARTESIAN_DYNAMICS
    try:
        robot.move(CartesianMotion(Affine([0.0, 0.0, 0.03]), ReferenceType.Relative))
        robot.move(CartesianMotion(Affine([0.0, 0.0, -0.03]), ReferenceType.Relative))
        print("  ✓ PASS: Z-axis motion")
    except Exception as e:
        print(f"  ✗ FAIL: {e}")
        robot.recover_from_errors()
        all_passed = False
    
    time.sleep(0.5)
    
    # Test 3: X-axis Cartesian (more challenging)
    print("\n[TEST 3] X-axis Cartesian Motion")
    robot.relative_dynamics_factor = CARTESIAN_DYNAMICS
    try:
        robot.move(CartesianMotion(Affine([0.03, 0.0, 0.0]), ReferenceType.Relative))
        robot.move(CartesianMotion(Affine([-0.03, 0.0, 0.0]), ReferenceType.Relative))
        print("  ✓ PASS: X-axis motion")
    except Exception as e:
        print(f"  ✗ FAIL: {e}")
        robot.recover_from_errors()
        all_passed = False
    
    time.sleep(0.5)
    
    # Test 4: Joint waypoints
    print("\n[TEST 4] Joint Waypoint Motion")
    robot.relative_dynamics_factor = JOINT_DYNAMICS
    try:
        current_q = list(robot.current_joint_state.position)
        q1 = current_q.copy()
        q2 = current_q.copy()
        q1[5] += 0.1
        q2[5] -= 0.1
        
        robot.move(JointWaypointMotion([
            JointWaypoint(q1),
            JointWaypoint(q2),
            JointWaypoint(current_q),
        ]))
        print("  ✓ PASS: Joint waypoint motion")
    except Exception as e:
        print(f"  ✗ FAIL: {e}")
        robot.recover_from_errors()
        all_passed = False
    
    time.sleep(0.5)
    
    # Test 5: Cartesian waypoints (challenging)
    print("\n[TEST 5] Cartesian Waypoint Motion")
    robot.relative_dynamics_factor = CARTESIAN_DYNAMICS
    try:
        robot.move(CartesianWaypointMotion([
            CartesianWaypoint(Affine([0.0, 0.0, 0.02]), ReferenceType.Relative),
            CartesianWaypoint(Affine([0.0, 0.0, -0.02]), ReferenceType.Relative),
        ]))
        print("  ✓ PASS: Cartesian waypoint motion")
    except Exception as e:
        print(f"  ✗ FAIL: {e}")
        robot.recover_from_errors()
        all_passed = False
    
    time.sleep(0.5)
    
    # Test 6: Gripper
    print("\n[TEST 6] Gripper Control")
    try:
        gripper = Gripper(ROBOT_IP)
        gripper.open(0.05)
        gripper.move(0.03, 0.05)
        gripper.open(0.05)
        print("  ✓ PASS: Gripper control")
    except Exception as e:
        print(f"  ✗ FAIL: {e}")
        all_passed = False
    
    # Summary
    print("\n" + "=" * 60)
    if all_passed:
        print("✓ ALL TESTS PASSED!")
    else:
        print("⚠️  SOME TESTS FAILED")
    print("=" * 60)
    
    print("""
JETSON FRANKY CONFIGURATION GUIDE:

Before running:
  1. Close all browsers (Chrome/Chromium)
  2. Run: sudo jetson_clocks
  3. Set governors: 
     for g in /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor; do
       echo performance | sudo tee $g
     done

Code configuration:
  from franky import Robot, RealtimeConfig, RelativeDynamicsFactor
  
  robot = Robot("172.16.0.2", realtime_config=RealtimeConfig.Ignore)
  
  # For joint motions (faster):
  robot.relative_dynamics_factor = RelativeDynamicsFactor(0.03, 0.05, 0.1)
  
  # For cartesian motions (slower, safer):
  robot.relative_dynamics_factor = RelativeDynamicsFactor(0.02, 0.03, 0.05)

Working features:
  ✓ JointMotion
  ✓ JointWaypointMotion  
  ✓ CartesianMotion (Z-axis)
  ✓ Gripper control
  
Reliability concerns (need very low dynamics):
  △ CartesianMotion (X, Y axes)
  △ CartesianWaypointMotion
  
Not recommended on Jetson:
  ✗ Async motion at 1kHz (use 500-800Hz if needed)
  ✗ Callbacks during motion (slows control loop)
  ✗ High dynamics factors
""")


if __name__ == "__main__":
    main()
