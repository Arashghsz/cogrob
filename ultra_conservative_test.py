#!/usr/bin/env python3
"""
Ultra-conservative test for Jetson AGX Orin
Uses extremely low dynamics to ensure success rate > 95%
"""

import time
from franky import (
    Affine, Robot, Gripper,
    CartesianMotion, JointMotion,
    CartesianWaypointMotion, CartesianWaypoint,
    ReferenceType, RealtimeConfig,
    RelativeDynamicsFactor,
)

ROBOT_IP = "172.16.0.2"

# Ultra-conservative: 1% velocity, 2% acceleration, 5% jerk
ULTRA_SLOW = RelativeDynamicsFactor(0.01, 0.02, 0.05)


def test_motion(robot, name, motion):
    """Test a motion and report success/failure"""
    print(f"\n{name}...")
    try:
        robot.move(motion)
        print(f"  ✓ SUCCESS")
        return True
    except Exception as e:
        error_str = str(e)
        # Extract success rate if present
        if "success_rate" in error_str:
            rate = error_str.split("success_rate:")[-1].strip()
            print(f"  ✗ FAILED (success_rate: {rate})")
        else:
            print(f"  ✗ FAILED: {e}")
        robot.recover_from_errors()
        return False


def main():
    print("=" * 60)
    print("ULTRA-CONSERVATIVE JETSON TEST")
    print("=" * 60)
    print(f"Dynamics: vel=1%, acc=2%, jerk=5%")
    
    robot = Robot(ROBOT_IP, realtime_config=RealtimeConfig.Ignore)
    robot.recover_from_errors()
    robot.relative_dynamics_factor = ULTRA_SLOW
    
    print("\n✓ Connected and configured")
    
    # Get initial state
    initial_q = list(robot.current_joint_state.position)
    print(f"Initial joints: {[round(q, 3) for q in initial_q]}")
    
    results = {}
    
    # Test 1: Tiny Cartesian motion (2cm)
    results["Cartesian 2cm X"] = test_motion(
        robot, "TEST 1: Cartesian 2cm X",
        CartesianMotion(Affine([0.02, 0.0, 0.0]), ReferenceType.Relative)
    )
    time.sleep(0.5)
    
    # Move back
    if results["Cartesian 2cm X"]:
        test_motion(robot, "  Return", 
            CartesianMotion(Affine([-0.02, 0.0, 0.0]), ReferenceType.Relative))
        time.sleep(0.5)
    
    # Test 2: Joint motion (0.05 rad only)
    target_q = initial_q.copy()
    target_q[0] += 0.05  # Half the previous test
    
    results["Joint 0.05 rad"] = test_motion(
        robot, "TEST 2: Joint 0.05 rad J1",
        JointMotion(target_q)
    )
    time.sleep(0.5)
    
    if results["Joint 0.05 rad"]:
        test_motion(robot, "  Return", JointMotion(initial_q))
        time.sleep(0.5)
    
    # Test 3: Very small Cartesian waypoints (1cm each)
    results["Waypoints 1cm"] = test_motion(
        robot, "TEST 3: Waypoints 1cm square",
        CartesianWaypointMotion([
            CartesianWaypoint(Affine([0.01, 0.0, 0.0]), ReferenceType.Relative),
            CartesianWaypoint(Affine([0.0, 0.01, 0.0]), ReferenceType.Relative),
            CartesianWaypoint(Affine([-0.01, 0.0, 0.0]), ReferenceType.Relative),
            CartesianWaypoint(Affine([0.0, -0.01, 0.0]), ReferenceType.Relative),
        ])
    )
    time.sleep(0.5)
    
    # Test 4: Z motion (2cm)
    results["Z 2cm"] = test_motion(
        robot, "TEST 4: Z-axis 2cm up",
        CartesianMotion(Affine([0.0, 0.0, 0.02]), ReferenceType.Relative)
    )
    time.sleep(0.5)
    
    if results["Z 2cm"]:
        test_motion(robot, "  Return",
            CartesianMotion(Affine([0.0, 0.0, -0.02]), ReferenceType.Relative))
    
    # Summary
    print("\n" + "=" * 60)
    print("RESULTS SUMMARY")
    print("=" * 60)
    passed = sum(1 for v in results.values() if v)
    total = len(results)
    print(f"\nPassed: {passed}/{total}")
    for name, success in results.items():
        status = "✓ PASS" if success else "✗ FAIL"
        print(f"  {status}: {name}")
    
    if passed < total:
        print("\n⚠️  Some tests failed. Consider:")
        print("  1. Lower dynamics even more")
        print("  2. Check thermal throttling: tegrastats")
        print("  3. Reduce system load")
        print("  4. Use libfranka directly for critical applications")
    else:
        print("\n✓ All tests passed!")
        print("  Safe to use these dynamics for your application")


if __name__ == "__main__":
    main()
