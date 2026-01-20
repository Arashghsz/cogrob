#!/usr/bin/env python3
"""
Fun Franka Dance Routine
=========================
Makes the robot do entertaining, repeatable movements!

Run with: python fun_dance.py
"""

import time
import math
from franky import (
    Affine, Robot, Gripper,
    CartesianMotion, JointMotion,
    JointWaypointMotion, JointWaypoint,
    ReferenceType, RealtimeConfig,
    RelativeDynamicsFactor,
)

ROBOT_IP = "172.16.0.2"

# Dynamics profiles (increase for faster movement, decrease if errors occur)
SLOW = RelativeDynamicsFactor(0.05, 0.08, 0.15)
MEDIUM = RelativeDynamicsFactor(0.08, 0.12, 0.2)
FAST = RelativeDynamicsFactor(0.12, 0.18, 0.25)


class FrankaDancer:
    def __init__(self, robot_ip: str):
        print("🤖 Connecting to Franka...")
        self.robot = Robot(robot_ip, realtime_config=RealtimeConfig.Ignore)
        self.robot.recover_from_errors()
        self.gripper = Gripper(robot_ip)
        self.home_position = None
        print("🎉 Ready to dance!")
    
    def save_home(self):
        """Save current position as home"""
        self.home_position = list(self.robot.current_joint_state.position)
        print(f"📍 Home saved")
    
    def go_home(self):
        """Return to home position"""
        if self.home_position:
            self.robot.relative_dynamics_factor = SLOW
            self._safe_move(JointMotion(self.home_position))
            print("🏠 Returned home")
    
    def _safe_move(self, motion, retries=3):
        """Move with retry on error"""
        for attempt in range(retries):
            try:
                self.robot.move(motion)
                return True
            except Exception as e:
                if attempt < retries - 1:
                    time.sleep(0.3)
                    self.robot.recover_from_errors()
                else:
                    print(f"⚠️  Motion failed: {e}")
                    self.robot.recover_from_errors()
                    return False
        return False
    
    def wave_hello(self):
        """Wave the gripper like saying hello"""
        print("👋 Waving hello!")
        self.robot.relative_dynamics_factor = FAST
        
        current = list(self.robot.current_joint_state.position)
        
        # Wave by rotating wrist back and forth
        for _ in range(3):
            wave_left = current.copy()
            wave_left[6] += 0.4  # Rotate wrist
            
            wave_right = current.copy()
            wave_right[6] -= 0.4
            
            self._safe_move(JointMotion(wave_left))
            self._safe_move(JointMotion(wave_right))
        
        self._safe_move(JointMotion(current))
    
    def nod_yes(self):
        """Nod up and down like saying yes"""
        print("✅ Nodding yes!")
        self.robot.relative_dynamics_factor = MEDIUM
        
        for _ in range(3):
            self._safe_move(CartesianMotion(Affine([0, 0, 0.03]), ReferenceType.Relative))
            self._safe_move(CartesianMotion(Affine([0, 0, -0.03]), ReferenceType.Relative))
    
    def shake_no(self):
        """Shake side to side like saying no"""
        print("❌ Shaking no!")
        self.robot.relative_dynamics_factor = MEDIUM
        
        for _ in range(3):
            self._safe_move(CartesianMotion(Affine([0, 0.03, 0]), ReferenceType.Relative))
            self._safe_move(CartesianMotion(Affine([0, -0.06, 0]), ReferenceType.Relative))
            self._safe_move(CartesianMotion(Affine([0, 0.03, 0]), ReferenceType.Relative))
    
    def spin_base(self):
        """Spin the robot base around"""
        print("🔄 Spinning around!")
        self.robot.relative_dynamics_factor = MEDIUM
        
        current = list(self.robot.current_joint_state.position)
        
        # Spin base joint (J1) around
        spin1 = current.copy()
        spin1[0] += 0.5  # Turn right
        
        spin2 = current.copy()
        spin2[0] -= 0.5  # Turn left
        
        self._safe_move(JointMotion(spin1))
        time.sleep(0.2)
        self._safe_move(JointMotion(spin2))
        time.sleep(0.2)
        self._safe_move(JointMotion(current))
    
    def do_the_robot(self):
        """Classic robot dance move - stiff jerky movements"""
        print("🤖 Doing the robot!")
        self.robot.relative_dynamics_factor = MEDIUM
        
        current = list(self.robot.current_joint_state.position)
        
        # Sequence of angular poses
        poses = [
            [0.1, 0, 0, 0, 0, 0.2, 0],   # Pose 1
            [-0.1, 0, 0, 0, 0, -0.2, 0],  # Pose 2
            [0, 0.1, 0, -0.1, 0, 0, 0.3], # Pose 3
            [0, -0.1, 0, 0.1, 0, 0, -0.3], # Pose 4
        ]
        
        for delta in poses:
            target = [c + d for c, d in zip(current, delta)]
            self._safe_move(JointMotion(target))
            time.sleep(0.1)
        
        self._safe_move(JointMotion(current))
    
    def gripper_clap(self):
        """Open and close gripper like clapping"""
        print("👏 Clapping!")
        
        for _ in range(5):
            self.gripper.move(0.08, 0.1)  # Open wide
            self.gripper.move(0.01, 0.1)  # Close
        
        self.gripper.open(0.05)
    
    def draw_circle(self):
        """Draw a circle in the air"""
        print("⭕ Drawing a circle!")
        self.robot.relative_dynamics_factor = MEDIUM
        
        # Approximate circle with 8 waypoints
        radius = 0.03  # 3cm radius
        steps = 8
        
        for i in range(steps + 1):
            angle = 2 * math.pi * i / steps
            x = radius * math.cos(angle) - radius  # Start at rightmost point
            z = radius * math.sin(angle)
            
            if i == 0:
                # Move to start position
                self._safe_move(CartesianMotion(Affine([0, 0, radius]), ReferenceType.Relative))
            else:
                # Small incremental moves
                prev_angle = 2 * math.pi * (i-1) / steps
                dx = radius * (math.cos(angle) - math.cos(prev_angle))
                dz = radius * (math.sin(angle) - math.sin(prev_angle))
                self._safe_move(CartesianMotion(Affine([0, 0, dz]), ReferenceType.Relative))
    
    def bow(self):
        """Take a bow"""
        print("🎭 Taking a bow!")
        self.robot.relative_dynamics_factor = MEDIUM
        
        current = list(self.robot.current_joint_state.position)
        
        # Bow by tilting forward
        bow_pose = current.copy()
        bow_pose[1] -= 0.2  # Tilt shoulder
        bow_pose[3] += 0.3  # Bend elbow
        
        self._safe_move(JointMotion(bow_pose))
        time.sleep(1.0)  # Hold the bow
        self._safe_move(JointMotion(current))
    
    def full_dance_routine(self):
        """Execute the full dance routine"""
        print("\n" + "=" * 50)
        print("🎵 FRANKA DANCE PARTY! 🎵")
        print("=" * 50 + "\n")
        
        self.save_home()
        
        routines = [
            ("Wave", self.wave_hello),
            ("Nod", self.nod_yes),
            ("Shake", self.shake_no),
            ("Spin", self.spin_base),
            ("Clap", self.gripper_clap),
            ("Robot Dance", self.do_the_robot),
            ("Bow", self.bow),
        ]
        
        for name, routine in routines:
            print(f"\n--- {name} ---")
            routine()
            time.sleep(0.5)
        
        self.go_home()
        
        print("\n" + "=" * 50)
        print("🎉 DANCE COMPLETE! 🎉")
        print("=" * 50 + "\n")


def main():
    dancer = FrankaDancer(ROBOT_IP)
    
    print("\n🎮 FRANKA FUN MENU")
    print("=" * 30)
    print("1. Full dance routine")
    print("2. Wave hello")
    print("3. Nod yes")
    print("4. Shake no")
    print("5. Spin around")
    print("6. Gripper clap")
    print("7. Do the robot")
    print("8. Draw circle")
    print("9. Take a bow")
    print("0. Exit")
    print("=" * 30)
    
    dancer.save_home()
    
    while True:
        try:
            choice = input("\nChoose activity (0-9): ").strip()
            
            if choice == "0":
                print("👋 Goodbye!")
                break
            elif choice == "1":
                dancer.full_dance_routine()
            elif choice == "2":
                dancer.wave_hello()
            elif choice == "3":
                dancer.nod_yes()
            elif choice == "4":
                dancer.shake_no()
            elif choice == "5":
                dancer.spin_base()
            elif choice == "6":
                dancer.gripper_clap()
            elif choice == "7":
                dancer.do_the_robot()
            elif choice == "8":
                dancer.draw_circle()
            elif choice == "9":
                dancer.bow()
            else:
                print("Invalid choice, try again")
                
        except KeyboardInterrupt:
            print("\n👋 Interrupted, going home...")
            dancer.go_home()
            break
    
    dancer.go_home()


if __name__ == "__main__":
    main()
