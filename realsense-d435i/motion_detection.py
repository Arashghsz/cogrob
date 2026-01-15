#!/usr/bin/env python3
"""
Motion detection using RealSense RGB camera
Highlights moving objects in real-time
"""

import cv2
import numpy as np
from datetime import datetime

def main():
    print("=" * 60)
    print("Motion Detection - RealSense RGB Camera")
    print("=" * 60)
    
    cap = cv2.VideoCapture(4)  # RGB camera
    
    if not cap.isOpened():
        print("❌ Cannot open camera")
        return
    
    # Get first frame
    ret, frame1 = cap.read()
    ret, frame2 = cap.read()
    
    if not ret:
        print("Cannot read frames")
        return
    
    print("\nCamera ready")
    print("\nControls:")
    print("  'q' - Quit")
    print("  's' - Save motion snapshot")
    print("  '+' - Increase sensitivity")
    print("  '-' - Decrease sensitivity\n")
    
    # Motion detection threshold
    threshold = 25
    min_area = 500  # Minimum area to consider as motion
    
    try:
        while True:
            # Calculate difference between frames
            diff = cv2.absdiff(frame1, frame2)
            gray = cv2.cvtColor(diff, cv2.COLOR_BGR2GRAY)
            blur = cv2.GaussianBlur(gray, (5, 5), 0)
            _, thresh = cv2.threshold(blur, threshold, 255, cv2.THRESH_BINARY)
            
            # Dilate to fill holes
            dilated = cv2.dilate(thresh, None, iterations=3)
            
            # Find contours
            contours, _ = cv2.findContours(dilated, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            
            # Draw on frame2
            motion_frame = frame2.copy()
            motion_detected = False
            
            for contour in contours:
                if cv2.contourArea(contour) < min_area:
                    continue
                
                motion_detected = True
                (x, y, w, h) = cv2.boundingRect(contour)
                cv2.rectangle(motion_frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.putText(motion_frame, "MOTION", (x, y - 10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
            # Add status
            status = "MOTION DETECTED!" if motion_detected else "No Motion"
            color = (0, 0, 255) if motion_detected else (0, 255, 0)
            cv2.putText(motion_frame, status, (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2)
            
            cv2.putText(motion_frame, f"Threshold: {threshold}", (10, 60),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            
            # Show frames
            cv2.imshow('Motion Detection', motion_frame)
            cv2.imshow('Motion Mask', dilated)
            
            # Shift frames
            frame1 = frame2
            ret, frame2 = cap.read()
            
            if not ret:
                break
            
            key = cv2.waitKey(10) & 0xFF
            if key == ord('q') or key == 27:
                break
            elif key == ord('s') and motion_detected:
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                filename = f"/home/arash/cogrob/motion_{timestamp}.jpg"
                cv2.imwrite(filename, motion_frame)
                print(f"Saved: {filename}")
            elif key == ord('+'):
                threshold = min(255, threshold + 5)
                print(f"Threshold: {threshold}")
            elif key == ord('-'):
                threshold = max(5, threshold - 5)
                print(f"Threshold: {threshold}")
    
    except KeyboardInterrupt:
        print("\nInterrupted")
    finally:
        cap.release()
        cv2.destroyAllWindows()
        print("Camera closed")

if __name__ == "__main__":
    main()
