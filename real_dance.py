#!/usr/bin/env python3
"""
Franka Real Dance - Joint-Based Dancing
=========================================
Complex dance moves using coordinated joint movements.
No simple Cartesian moves - real dancing with all joints!

Run with: python real_dance.py
"""

import time
import math
import signal
import sys
from franky import (
    Robot, Gripper,
    JointMotion,
    JointWaypointMotion, JointWaypoint,
    RealtimeConfig,
    RelativeDynamicsFactor,
)

ROBOT_IP = "172.16.0.2"

# Speed profiles
SLOW = RelativeDynamicsFactor(0.05, 0.08, 0.15)
MEDIUM = RelativeDynamicsFactor(0.08, 0.12, 0.2)
FAST = RelativeDynamicsFactor(0.12, 0.18, 0.25)

running = True


def signal_handler(sig, frame):
    global running
    print("\n🛑 Stopping dance...")
    running = False


class FrankaDancer:
    """Real dancing using coordinated joint movements"""
    
    def __init__(self, robot_ip: str):
        print("🤖 Connecting to Franka dancer...")
        self.robot = Robot(robot_ip, realtime_config=RealtimeConfig.Ignore)
        self.robot.recover_from_errors()
        self.gripper = Gripper(robot_ip)
        self.home = list(self.robot.current_joint_state.position)
        self.robot.relative_dynamics_factor = MEDIUM
        print("💃 Ready to dance!")
    
    def safe_move(self, motion, retries=3):
        for attempt in range(retries):
            try:
                self.robot.move(motion)
                return True
            except:
                time.sleep(0.2)
                self.robot.recover_from_errors()
        return False
    
    def get_joints(self):
        return list(self.robot.current_joint_state.position)
    
    def go_home(self):
        self.safe_move(JointMotion(self.home))
    
    # ============ DANCE MOVES ============
    
    def disco_point(self):
        """Classic disco pointing move - arm up and down"""
        print("  🕺 Disco point!")
        self.robot.relative_dynamics_factor = FAST
        
        q = self.get_joints()
        
        # Point up right
        up_right = q.copy()
        up_right[0] += 0.3   # Base right
        up_right[1] -= 0.2   # Shoulder up
        up_right[3] += 0.3   # Elbow out
        up_right[5] -= 0.2   # Wrist angle
        
        # Point up left  
        up_left = q.copy()
        up_left[0] -= 0.3    # Base left
        up_left[1] -= 0.2    # Shoulder up
        up_left[3] += 0.3    # Elbow out
        up_left[5] -= 0.2    # Wrist angle
        
        # Down center
        down = q.copy()
        down[1] += 0.1
        down[3] -= 0.2
        
        for _ in range(2):
            self.safe_move(JointMotion(up_right))
            self.safe_move(JointMotion(down))
            self.safe_move(JointMotion(up_left))
            self.safe_move(JointMotion(down))
        
        self.safe_move(JointMotion(q))
    
    def snake_wave(self):
        """Snake-like wave through all joints"""
        print("  🐍 Snake wave!")
        self.robot.relative_dynamics_factor = MEDIUM
        
        q = self.get_joints()
        
        # Create a wave that travels through joints
        waypoints = []
        steps = 8
        
        for i in range(steps):
            pose = q.copy()
            phase = 2 * math.pi * i / steps
            
            # Each joint waves with a phase offset
            pose[0] += 0.15 * math.sin(phase)           # Base
            pose[1] += 0.1 * math.sin(phase + 0.5)      # Shoulder
            pose[3] += 0.15 * math.sin(phase + 1.0)     # Elbow
            pose[5] += 0.2 * math.sin(phase + 1.5)      # Wrist1
            pose[6] += 0.25 * math.sin(phase + 2.0)     # Wrist2
            
            waypoints.append(JointWaypoint(pose))
        
        waypoints.append(JointWaypoint(q))
        self.safe_move(JointWaypointMotion(waypoints))
    
    def robot_groove(self):
        """Groovy robot dance with sharp angles"""
        print("  🤖 Robot groove!")
        self.robot.relative_dynamics_factor = FAST
        
        q = self.get_joints()
        
        poses = [
            # Pose 1: Right angle
            [0.2, -0.1, 0, 0.2, 0, -0.2, 0.3],
            # Pose 2: Left angle
            [-0.2, 0.1, 0, -0.2, 0, 0.2, -0.3],
            # Pose 3: Up pose
            [0, -0.15, 0.1, 0.3, -0.1, -0.3, 0],
            # Pose 4: Down pose
            [0, 0.15, -0.1, -0.3, 0.1, 0.3, 0],
        ]
        
        for _ in range(2):
            for delta in poses:
                target = [c + d for c, d in zip(q, delta)]
                self.safe_move(JointMotion(target))
        
        self.safe_move(JointMotion(q))
    
    def hip_hop_bounce(self):
        """Hip hop style bounce with attitude"""
        print("  🎤 Hip hop bounce!")
        self.robot.relative_dynamics_factor = FAST
        
        q = self.get_joints()
        
        # Bounce poses
        up = q.copy()
        up[1] -= 0.15  # Shoulder up
        up[3] += 0.2   # Elbow bent
        up[5] -= 0.15  # Wrist cocked
        
        down = q.copy()
        down[1] += 0.1  # Shoulder down
        down[3] -= 0.15 # Elbow straight
        down[5] += 0.1  # Wrist down
        
        # Bounce with attitude (add base rotation)
        for i in range(4):
            up_twist = up.copy()
            up_twist[0] += 0.15 * (1 if i % 2 == 0 else -1)
            up_twist[6] += 0.2 * (1 if i % 2 == 0 else -1)
            
            self.safe_move(JointMotion(up_twist))
            self.safe_move(JointMotion(down))
        
        self.safe_move(JointMotion(q))
    
    def arm_circle(self):
        """Circular arm motion using joints"""
        print("  🔵 Arm circle!")
        self.robot.relative_dynamics_factor = MEDIUM
        
        q = self.get_joints()
        
        waypoints = []
        steps = 12
        
        for i in range(steps + 1):
            pose = q.copy()
            angle = 2 * math.pi * i / steps
            
            # Create circular motion through shoulder and elbow
            pose[1] += 0.12 * math.cos(angle)  # Shoulder
            pose[3] += 0.15 * math.sin(angle)  # Elbow
            pose[5] += 0.1 * math.cos(angle)   # Wrist follows
            
            waypoints.append(JointWaypoint(pose))
        
        self.safe_move(JointWaypointMotion(waypoints))
    
    def salsa_twist(self):
        """Salsa-style twisting motion"""
        print("  💃 Salsa twist!")
        self.robot.relative_dynamics_factor = FAST
        
        q = self.get_joints()
        
        for _ in range(3):
            # Twist right
            right = q.copy()
            right[0] += 0.25
            right[2] += 0.15
            right[4] += 0.2
            right[6] += 0.3
            self.safe_move(JointMotion(right))
            
            # Twist left
            left = q.copy()
            left[0] -= 0.25
            left[2] -= 0.15
            left[4] -= 0.2
            left[6] -= 0.3
            self.safe_move(JointMotion(left))
        
        self.safe_move(JointMotion(q))
    
    def wave_goodbye(self):
        """Enthusiastic wave using wrist and elbow"""
        print("  👋 Wave goodbye!")
        self.robot.relative_dynamics_factor = FAST
        
        q = self.get_joints()
        
        # Raise arm first
        raised = q.copy()
        raised[1] -= 0.2
        raised[3] += 0.25
        self.safe_move(JointMotion(raised))
        
        # Wave with wrist
        for _ in range(4):
            wave_r = raised.copy()
            wave_r[5] += 0.25
            wave_r[6] += 0.35
            self.safe_move(JointMotion(wave_r))
            
            wave_l = raised.copy()
            wave_l[5] -= 0.25
            wave_l[6] -= 0.35
            self.safe_move(JointMotion(wave_l))
        
        self.safe_move(JointMotion(q))
    
    def shimmy(self):
        """Quick shimmy shake"""
        print("  ✨ Shimmy!")
        self.robot.relative_dynamics_factor = FAST
        
        q = self.get_joints()
        
        for _ in range(6):
            left = q.copy()
            left[0] -= 0.1
            left[2] -= 0.08
            left[4] -= 0.1
            self.safe_move(JointMotion(left))
            
            right = q.copy()
            right[0] += 0.1
            right[2] += 0.08
            right[4] += 0.1
            self.safe_move(JointMotion(right))
        
        self.safe_move(JointMotion(q))
    
    def dramatic_bow(self):
        """Grand theatrical bow"""
        print("  🎭 Dramatic bow!")
        self.robot.relative_dynamics_factor = MEDIUM
        
        q = self.get_joints()
        
        # Rise up first
        up = q.copy()
        up[1] -= 0.15
        up[3] += 0.2
        self.safe_move(JointMotion(up))
        
        # Deep bow
        bow = q.copy()
        bow[1] += 0.25   # Shoulder forward
        bow[3] -= 0.35   # Elbow extended
        bow[5] += 0.2    # Wrist down
        self.safe_move(JointMotion(bow))
        
        time.sleep(0.8)  # Hold the bow
        
        # Rise back up gracefully
        self.safe_move(JointMotion(up))
        self.safe_move(JointMotion(q))
    
    def full_routine(self):
        """Complete dance routine"""
        moves = [
            self.disco_point,
            self.snake_wave,
            self.robot_groove,
            self.hip_hop_bounce,
            self.arm_circle,
            self.salsa_twist,
            self.shimmy,
            self.wave_goodbye,
            self.dramatic_bow,
        ]
        
        for move in moves:
            if not running:
                break
            move()
            time.sleep(0.3)


def main():
    global running
    signal.signal(signal.SIGINT, signal_handler)
    
    print("=" * 50)
    print("💃 FRANKA REAL DANCE 🕺")
    print("=" * 50)
    print("\nAll moves use coordinated joint movements!")
    print("Press Ctrl+C to stop\n")
    
    dancer = FrankaDancer(ROBOT_IP)
    
    # Gripper clap to start
    print("👏 Starting clap!")
    for _ in range(3):
        dancer.gripper.move(0.08, 0.15)
        dancer.gripper.move(0.01, 0.15)
    dancer.gripper.open(0.08)
    
    loop = 0
    while running:
        loop += 1
        print(f"\n🎵 === DANCE LOOP {loop} === 🎵")
        dancer.full_routine()
        
        if running:
            dancer.go_home()
            time.sleep(0.5)
    
    print("\n🏠 Going home...")
    dancer.go_home()
    print("👋 Dance complete!")


if __name__ == "__main__":
    main()
