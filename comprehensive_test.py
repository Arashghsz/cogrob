#!/usr/bin/env python3
"""
Comprehensive Franky Test Suite
================================
Tests basic to advanced features to verify franky installation.

System info:
- libfranka: 0.18.0
- franky-control: 1.0.1 
- Platform: Jetson AGX Orin (aarch64)
"""

import sys
import time

# Test 1: Basic imports
print("=" * 60)
print("TEST 1: Basic Imports")
print("=" * 60)
try:
    from franky import (
        Affine, Robot, Gripper,
        CartesianMotion, JointMotion,
        CartesianWaypointMotion, CartesianWaypoint,
        JointWaypointMotion, JointWaypoint,
        JointVelocityMotion,
        ReferenceType, RealtimeConfig,
        Duration, Measure, Reaction,
        RelativeDynamicsFactor,
    )
    print("✓ All imports successful")
except ImportError as e:
    print(f"✗ Import failed: {e}")
    sys.exit(1)

# Test 2: Affine creation (was causing segfaults in some versions)
print("\n" + "=" * 60)
print("TEST 2: Affine Object Creation")
print("=" * 60)
try:
    a1 = Affine()
    print(f"✓ Affine() = {a1}")
    
    a2 = Affine([0.1, 0.2, 0.3])
    print(f"✓ Affine([0.1, 0.2, 0.3]) = {a2}")
    
    a3 = Affine([0.0, 0.0, 0.5], [0, 0, 0, 1])  # with quaternion
    print(f"✓ Affine with quaternion = {a3}")
    
    # Test translation access
    print(f"✓ Translation: {a2.translation}")
except Exception as e:
    print(f"✗ Affine test failed: {e}")

# Test 3: Motion object creation
print("\n" + "=" * 60)
print("TEST 3: Motion Object Creation")
print("=" * 60)
try:
    # Cartesian motion
    cm = CartesianMotion(Affine([0.05, 0.0, 0.0]), ReferenceType.Relative)
    print("✓ CartesianMotion created")
    
    # Joint motion
    jm = JointMotion([0.0, 0.1, 0.0, -1.5, 0.0, 1.8, 0.0])
    print("✓ JointMotion created")
    
    # Waypoint motion
    wm = CartesianWaypointMotion([
        CartesianWaypoint(Affine([0.05, 0.0, 0.0]), ReferenceType.Relative),
        CartesianWaypoint(Affine([-0.05, 0.0, 0.0]), ReferenceType.Relative),
    ])
    print("✓ CartesianWaypointMotion created")
    
except Exception as e:
    print(f"✗ Motion creation failed: {e}")

# Test 4: Robot connection
print("\n" + "=" * 60)
print("TEST 4: Robot Connection")
print("=" * 60)
ROBOT_IP = "172.16.0.2"
try:
    robot = Robot(ROBOT_IP, realtime_config=RealtimeConfig.Ignore)
    print(f"✓ Connected to robot at {ROBOT_IP}")
    
    # Recover from errors
    robot.recover_from_errors()
    print("✓ Recovered from errors")
    
except Exception as e:
    print(f"✗ Robot connection failed: {e}")
    sys.exit(1)

# Test 5: Read robot state
print("\n" + "=" * 60)
print("TEST 5: Robot State Reading")
print("=" * 60)
try:
    state = robot.state
    print(f"✓ Got robot state")
    
    joint_state = robot.current_joint_state
    print(f"✓ Joint positions: {[round(p, 3) for p in joint_state.position]}")
    print(f"✓ Joint velocities: {[round(v, 4) for v in joint_state.velocity]}")
    
    cartesian_state = robot.current_cartesian_state
    ee_pose = cartesian_state.pose.end_effector_pose
    print(f"✓ EE position: x={ee_pose.translation[0]:.3f}, y={ee_pose.translation[1]:.3f}, z={ee_pose.translation[2]:.3f}")
    
except Exception as e:
    print(f"✗ State reading failed: {e}")

# Test 6: Simple Cartesian motion
print("\n" + "=" * 60)
print("TEST 6: Simple Cartesian Motion (5cm X)")
print("=" * 60)
try:
    robot.relative_dynamics_factor = 0.02  # Very slow for Jetson
    
    motion = CartesianMotion(Affine([0.05, 0.0, 0.0]), ReferenceType.Relative)
    robot.move(motion)
    print("✓ Cartesian motion forward completed")
    
    time.sleep(0.5)
    
    # Move back
    motion_back = CartesianMotion(Affine([-0.05, 0.0, 0.0]), ReferenceType.Relative)
    robot.move(motion_back)
    print("✓ Cartesian motion back completed")
    
except Exception as e:
    print(f"✗ Cartesian motion failed: {e}")
    robot.recover_from_errors()

# Test 7: Joint motion
print("\n" + "=" * 60)
print("TEST 7: Joint Motion")
print("=" * 60)
try:
    # Get current joint positions
    current_q = list(robot.current_joint_state.position)
    print(f"  Current joints: {[round(q, 3) for q in current_q]}")
    
    # Small joint motion (move joint 0 by 0.1 rad)
    target_q = current_q.copy()
    target_q[0] += 0.1
    
    robot.relative_dynamics_factor = 0.02
    jm = JointMotion(target_q)
    robot.move(jm)
    print("✓ Joint motion forward completed")
    
    time.sleep(0.5)
    
    # Move back
    jm_back = JointMotion(current_q)
    robot.move(jm_back)
    print("✓ Joint motion back completed")
    
except Exception as e:
    print(f"✗ Joint motion failed: {e}")
    robot.recover_from_errors()

# Test 8: Waypoint motion
print("\n" + "=" * 60)
print("TEST 8: Multi-Waypoint Motion")
print("=" * 60)
try:
    robot.relative_dynamics_factor = 0.02
    
    waypoint_motion = CartesianWaypointMotion([
        CartesianWaypoint(Affine([0.03, 0.0, 0.0]), ReferenceType.Relative),
        CartesianWaypoint(Affine([0.0, 0.03, 0.0]), ReferenceType.Relative),
        CartesianWaypoint(Affine([-0.03, 0.0, 0.0]), ReferenceType.Relative),
        CartesianWaypoint(Affine([0.0, -0.03, 0.0]), ReferenceType.Relative),
    ])
    robot.move(waypoint_motion)
    print("✓ Waypoint motion (square) completed")
    
except Exception as e:
    print(f"✗ Waypoint motion failed: {e}")
    robot.recover_from_errors()

# Test 9: Dynamics factor adjustment
print("\n" + "=" * 60)
print("TEST 9: RelativeDynamicsFactor")
print("=" * 60)
try:
    # Set individual factors
    robot.relative_dynamics_factor = RelativeDynamicsFactor(0.03, 0.05, 0.1)
    print("✓ Set RelativeDynamicsFactor(vel=0.03, acc=0.05, jerk=0.1)")
    
    motion = CartesianMotion(Affine([0.03, 0.0, 0.0]), ReferenceType.Relative)
    robot.move(motion)
    print("✓ Motion with custom dynamics completed")
    
    # Move back
    motion_back = CartesianMotion(Affine([-0.03, 0.0, 0.0]), ReferenceType.Relative)
    robot.move(motion_back)
    print("✓ Motion back completed")
    
    # Reset to simple factor
    robot.relative_dynamics_factor = 0.02
    
except Exception as e:
    print(f"✗ Dynamics factor test failed: {e}")
    robot.recover_from_errors()

# Test 10: Gripper (if available)
print("\n" + "=" * 60)
print("TEST 10: Gripper Control")
print("=" * 60)
try:
    gripper = Gripper(ROBOT_IP)
    print(f"✓ Connected to gripper")
    
    width = gripper.width
    print(f"✓ Current gripper width: {width * 1000:.1f} mm")
    
    # Open gripper
    gripper.open(0.05)
    print(f"✓ Gripper opened, width: {gripper.width * 1000:.1f} mm")
    
    # Close to specific width
    gripper.move(0.04, 0.05)  # 4cm width, 5cm/s speed
    print(f"✓ Gripper moved to 40mm, width: {gripper.width * 1000:.1f} mm")
    
    # Open again
    gripper.open(0.05)
    print(f"✓ Gripper opened again")
    
except Exception as e:
    print(f"✗ Gripper test failed: {e}")

# Summary
print("\n" + "=" * 60)
print("TEST SUMMARY")
print("=" * 60)
print("""
System Configuration:
- libfranka: 0.18.0 (installed at /usr/local/lib/)
- franky-control: 1.0.1
- Platform: Jetson AGX Orin
- RealtimeConfig: Ignore (for Jetson compatibility)
- Dynamics factor: 0.02 (2% for safe Jetson operation)

Key findings from GitHub issues:
- Issue #47: Discontinuity errors can occur with weak hardware
- Issue #46: Async control works best at 500-800Hz, not 1000Hz
- Issue #18: NumPy 2.x can cause segfaults (use NumPy 1.x)
- Issue #24: Version 1.0.1 fixed segfaults in robot.move()

Recommendations for Jetson:
1. Run 'sudo jetson_clocks' before use
2. Use RealtimeConfig.Ignore
3. Keep dynamics factor low (0.02-0.05)
4. Avoid callbacks during critical motions
""")
