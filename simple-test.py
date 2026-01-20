from franky import Affine, CartesianMotion, Robot, ReferenceType

robot = Robot("172.16.0.2")
robot.relative_dynamics_factor = 0.05

motion = CartesianMotion(Affine([0.2, 0.02, 0.02]), ReferenceType.Relative)
robot.move(motion)