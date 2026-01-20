from franky import *

# Set realtime config to ignore errors (for testing on non-ideal systems)
robot = Robot("172.16.0.2", realtime_config=RealtimeConfig.Ignore)

# Recover from any previous errors first
robot.recover_from_errors()

# Start VERY slow - Jetson may have real-time latency issues
# Use 1-2% of max dynamics for testing
robot.relative_dynamics_factor = 0.02

# Move a small distance first (2cm instead of 50cm)
motion = CartesianMotion(Affine([0.02, -0.02, 0.0]), ReferenceType.Relative)

try:
    robot.move(motion)
    print("Motion completed successfully!")
except Exception as e:
    print(f"Motion failed: {e}")
    robot.recover_from_errors()