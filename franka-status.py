from franka_desk import FrankaDesk

desk = FrankaDesk()
desk.connect() 

current_joints = desk.joint_motion.get_joint_positions()
print("Current joint positions:", current_joints)

current_cart = desk.cart_motion.get_cartesian_position()
print("Current Cartesian position:", current_cart)

gripper_status = desk.gripper.status()
print("Gripper status:", gripper_status)

safety_status = desk.safety.get_safety_status()
print("Safety status:", safety_status)  