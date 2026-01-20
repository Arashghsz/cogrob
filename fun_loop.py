#!/usr/bin/env python3
"""
Franka Continuous Fun Loop
===========================
Robot continuously performs fun activities in a loop.
Press Ctrl+C to stop.

Run with: python fun_loop.py
"""

import time
import signal
import sys
from franky import (
    Affine, Robot, Gripper,
    CartesianMotion, JointMotion,
    ReferenceType, RealtimeConfig,
    RelativeDynamicsFactor,
)

ROBOT_IP = "172.16.0.2"

# Dynamics profiles (increase for faster, decrease if errors occur)
SLOW = RelativeDynamicsFactor(0.05, 0.08, 0.15)
MEDIUM = RelativeDynamicsFactor(0.08, 0.12, 0.2)
FAST = RelativeDynamicsFactor(0.12, 0.18, 0.25)

# Global flag for clean shutdown
running = True


def signal_handler(sig, frame):
    global running
    print("\n🛑 Stopping...")
    running = False


class FrankaFun:
    def __init__(self, robot_ip: str):
        self.robot = Robot(robot_ip, realtime_config=RealtimeConfig.Ignore)
        self.robot.recover_from_errors()
        self.gripper = Gripper(robot_ip)
        self.home = list(self.robot.current_joint_state.position)
        self.robot.relative_dynamics_factor = MEDIUM
    
    def safe_move(self, motion):
        for _ in range(3):
            try:
                self.robot.move(motion)
                return True
            except:
                time.sleep(0.2)
                self.robot.recover_from_errors()
        return False
    
    def wave(self):
        """Wave wrist"""
        print("  👋 Wave")
        q = list(self.robot.current_joint_state.position)
        for _ in range(2):
            q[6] += 0.3
            self.safe_move(JointMotion(q))
            q[6] -= 0.6
            self.safe_move(JointMotion(q))
            q[6] += 0.3
            self.safe_move(JointMotion(q))
    
    def bob(self):
        """Bob up and down"""
        print("  ⬆️  Bob")
        for _ in range(2):
            self.safe_move(CartesianMotion(Affine([0, 0, 0.03]), ReferenceType.Relative))
            self.safe_move(CartesianMotion(Affine([0, 0, -0.03]), ReferenceType.Relative))
    
    def spin(self):
        """Spin base"""
        print("  🔄 Spin")
        q = list(self.robot.current_joint_state.position)
        orig_q0 = q[0]
        q[0] += 0.4
        self.safe_move(JointMotion(q))
        q[0] -= 0.8
        self.safe_move(JointMotion(q))
        q[0] = orig_q0
        self.safe_move(JointMotion(q))
    
    def sway(self):
        """Sway side to side"""
        print("  ↔️  Sway")
        for _ in range(2):
            self.safe_move(CartesianMotion(Affine([0, 0.03, 0]), ReferenceType.Relative))
            self.safe_move(CartesianMotion(Affine([0, -0.06, 0]), ReferenceType.Relative))
            self.safe_move(CartesianMotion(Affine([0, 0.03, 0]), ReferenceType.Relative))
    
    def clap(self):
        """Clap gripper"""
        print("  👏 Clap")
        for _ in range(4):
            self.gripper.move(0.07, 0.1)
            self.gripper.move(0.01, 0.1)
        self.gripper.open(0.05)
    
    def peek(self):
        """Peek forward and back"""
        print("  👀 Peek")
        self.safe_move(CartesianMotion(Affine([0.04, 0, 0]), ReferenceType.Relative))
        time.sleep(0.3)
        self.safe_move(CartesianMotion(Affine([-0.04, 0, 0]), ReferenceType.Relative))
    
    def twist(self):
        """Twist the arm"""
        print("  🌀 Twist")
        q = list(self.robot.current_joint_state.position)
        q[3] -= 0.2
        q[5] += 0.2
        self.safe_move(JointMotion(q))
        q[3] += 0.4
        q[5] -= 0.4
        self.safe_move(JointMotion(q))
        q[3] -= 0.2
        q[5] += 0.2
        self.safe_move(JointMotion(q))
    
    def go_home(self):
        print("  🏠 Home")
        self.safe_move(JointMotion(self.home))


def main():
    global running
    signal.signal(signal.SIGINT, signal_handler)
    
    print("=" * 50)
    print("🎵 FRANKA CONTINUOUS FUN LOOP 🎵")
    print("=" * 50)
    print("Press Ctrl+C to stop\n")
    
    franka = FrankaFun(ROBOT_IP)
    
    moves = [
        franka.wave,
        franka.bob,
        franka.spin,
        franka.clap,
        franka.sway,
        franka.peek,
        franka.twist,
    ]
    
    loop_count = 0
    move_index = 0
    
    while running:
        loop_count += 1
        print(f"\n🔁 Loop {loop_count}")
        
        # Do 3-4 moves per loop
        for _ in range(4):
            if not running:
                break
            moves[move_index]()
            move_index = (move_index + 1) % len(moves)
            time.sleep(0.3)
        
        if running:
            franka.go_home()
            time.sleep(0.5)
    
    # Clean exit
    print("\n🛑 Stopping and going home...")
    franka.go_home()
    print("👋 Done!")


if __name__ == "__main__":
    main()
