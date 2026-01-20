#!/usr/bin/env python3
"""
Optimized Franky Configuration for Jetson AGX Orin
===================================================

Key findings:
- Simple dynamics_factor (0.02) doesn't reduce jerk enough
- Must use RelativeDynamicsFactor(vel, acc, jerk) with LOW JERK
- Jerk is the critical parameter for Jetson real-time performance

Working configuration:
  robot.relative_dynamics_factor = RelativeDynamicsFactor(0.03, 0.05, 0.1)
  
This gives:
- 3% velocity (slow movements)
- 5% acceleration 
- 10% jerk (critical for discontinuity prevention)
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

# Optimized dynamics for Jetson - low jerk is critical!
JETSON_DYNAMICS = RelativeDynamicsFactor(0.03, 0.05, 0.1)


def main():
    print("Connecting to robot...")
    robot = Robot(ROBOT_IP, realtime_config=RealtimeConfig.Ignore)
    robot.recover_from_errors()
    
    # CRITICAL: Use explicit RelativeDynamicsFactor with low jerk
    robot.relative_dynamics_factor = JETSON_DYNAMICS
    print(f"✓ Connected with Jetson-optimized dynamics")
    
    # Test 1: Cartesian Motion
    print("\n[TEST 1] Cartesian Motion (5cm X)")
    try:
        motion = CartesianMotion(Affine([0.05, 0.0, 0.0]), ReferenceType.Relative)
        robot.move(motion)
        print("  ✓ Forward motion completed")
        
        time.sleep(0.3)
        
        motion_back = CartesianMotion(Affine([-0.05, 0.0, 0.0]), ReferenceType.Relative)
        robot.move(motion_back)
        print("  ✓ Return motion completed")
    except Exception as e:
        print(f"  ✗ Failed: {e}")
        robot.recover_from_errors()
    
    # Test 2: Joint Motion  
    print("\n[TEST 2] Joint Motion (0.1 rad on J1)")
    try:
        current_q = list(robot.current_joint_state.position)
        target_q = current_q.copy()
        target_q[0] += 0.1
        
        jm = JointMotion(target_q)
        robot.move(jm)
        print("  ✓ Forward motion completed")
        
        time.sleep(0.3)
        
        jm_back = JointMotion(current_q)
        robot.move(jm_back)
        print("  ✓ Return motion completed")
    except Exception as e:
        print(f"  ✗ Failed: {e}")
        robot.recover_from_errors()
    
    # Test 3: Cartesian Waypoint Motion
    print("\n[TEST 3] Cartesian Waypoint Motion (square pattern)")
    try:
        waypoints = CartesianWaypointMotion([
            CartesianWaypoint(Affine([0.03, 0.0, 0.0]), ReferenceType.Relative),
            CartesianWaypoint(Affine([0.0, 0.03, 0.0]), ReferenceType.Relative),
            CartesianWaypoint(Affine([-0.03, 0.0, 0.0]), ReferenceType.Relative),
            CartesianWaypoint(Affine([0.0, -0.03, 0.0]), ReferenceType.Relative),
        ])
        robot.move(waypoints)
        print("  ✓ Square pattern completed")
    except Exception as e:
        print(f"  ✗ Failed: {e}")
        robot.recover_from_errors()
    
    # Test 4: Joint Waypoint Motion
    print("\n[TEST 4] Joint Waypoint Motion")
    try:
        current_q = list(robot.current_joint_state.position)
        q1 = current_q.copy()
        q2 = current_q.copy()
        q1[5] += 0.1  # Move wrist slightly
        q2[5] -= 0.1
        
        waypoints = JointWaypointMotion([
            JointWaypoint(q1),
            JointWaypoint(q2),
            JointWaypoint(current_q),
        ])
        robot.move(waypoints)
        print("  ✓ Joint waypoint motion completed")
    except Exception as e:
        print(f"  ✗ Failed: {e}")
        robot.recover_from_errors()
    
    # Test 5: Z-axis motion
    print("\n[TEST 5] Z-axis Motion (3cm up/down)")
    try:
        motion_up = CartesianMotion(Affine([0.0, 0.0, 0.03]), ReferenceType.Relative)
        robot.move(motion_up)
        print("  ✓ Up motion completed")
        
        time.sleep(0.3)
        
        motion_down = CartesianMotion(Affine([0.0, 0.0, -0.03]), ReferenceType.Relative)
        robot.move(motion_down)
        print("  ✓ Down motion completed")
    except Exception as e:
        print(f"  ✗ Failed: {e}")
        robot.recover_from_errors()
    
    # Test 6: Gripper
    print("\n[TEST 6] Gripper Control")
    try:
        gripper = Gripper(ROBOT_IP)
        print(f"  Current width: {gripper.width * 1000:.1f} mm")
        
        gripper.open(0.05)
        print(f"  ✓ Opened to: {gripper.width * 1000:.1f} mm")
        
        gripper.move(0.02, 0.05)  # 2cm width
        print(f"  ✓ Closed to: {gripper.width * 1000:.1f} mm")
        
        gripper.open(0.05)
        print(f"  ✓ Reopened to: {gripper.width * 1000:.1f} mm")
    except Exception as e:
        print(f"  ✗ Failed: {e}")
    
    print("\n" + "=" * 50)
    print("ALL TESTS COMPLETED")
    print("=" * 50)
    print("""
JETSON CONFIGURATION SUMMARY:
  
  from franky import RelativeDynamicsFactor, RealtimeConfig
  
  robot = Robot(ip, realtime_config=RealtimeConfig.Ignore)
  robot.relative_dynamics_factor = RelativeDynamicsFactor(0.03, 0.05, 0.1)
  
  # The RelativeDynamicsFactor parameters are:
  #   - velocity factor: 0.03 (3% of max velocity)
  #   - acceleration factor: 0.05 (5% of max acceleration)  
  #   - jerk factor: 0.1 (10% of max jerk) <- CRITICAL for Jetson!
  
  # Don't forget to run: sudo jetson_clocks
""")


if __name__ == "__main__":
    main()
