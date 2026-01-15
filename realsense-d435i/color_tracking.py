#!/usr/bin/env python3
"""
Color-based object tracking
Track objects by color (useful for robotics)
"""

import cv2
import numpy as np

def main():
    print("=" * 60)
    print("Color Object Tracking - RealSense RGB Camera")
    print("=" * 60)
    
    cap = cv2.VideoCapture(4)
    
    if not cap.isOpened():
        print("Cannot open camera")
        return
    
    print("\nCamera ready")
    print("\nSelect color to track:")
    print("  1 - Red")
    print("  2 - Green")
    print("  3 - Blue")
    print("  4 - Yellow")
    print("  5 - Custom (adjust with trackbars)")
    
    choice = input("\nChoice (1-5): ").strip()
    
    # Color ranges in HSV
    color_ranges = {
        '1': ('Red', [(0, 100, 100), (10, 255, 255)], [(160, 100, 100), (180, 255, 255)]),
        '2': ('Green', [(40, 40, 40), (80, 255, 255)], None),
        '3': ('Blue', [(100, 100, 100), (130, 255, 255)], None),
        '4': ('Yellow', [(20, 100, 100), (30, 255, 255)], None),
        '5': ('Custom', [(0, 50, 50), (180, 255, 255)], None)
    }
    
    if choice not in color_ranges:
        choice = '1'
    
    color_name, range1, range2 = color_ranges[choice]
    
    print(f"\nTracking: {color_name}")
    print("\nControls:")
    print("  'q' - Quit")
    print("  'r' - Reset tracking")
    
    # Create window for trackbars if custom
    if choice == '5':
        cv2.namedWindow('Trackbars')
        cv2.createTrackbar('H Low', 'Trackbars', 0, 180, lambda x: None)
        cv2.createTrackbar('H High', 'Trackbars', 180, 180, lambda x: None)
        cv2.createTrackbar('S Low', 'Trackbars', 50, 255, lambda x: None)
        cv2.createTrackbar('S High', 'Trackbars', 255, 255, lambda x: None)
        cv2.createTrackbar('V Low', 'Trackbars', 50, 255, lambda x: None)
        cv2.createTrackbar('V High', 'Trackbars', 255, 255, lambda x: None)
    
    # For tracking history
    center_points = []
    max_points = 50
    
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                break
            
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            
            # Get trackbar values if custom
            if choice == '5':
                h_low = cv2.getTrackbarPos('H Low', 'Trackbars')
                h_high = cv2.getTrackbarPos('H High', 'Trackbars')
                s_low = cv2.getTrackbarPos('S Low', 'Trackbars')
                s_high = cv2.getTrackbarPos('S High', 'Trackbars')
                v_low = cv2.getTrackbarPos('V Low', 'Trackbars')
                v_high = cv2.getTrackbarPos('V High', 'Trackbars')
                
                lower = np.array([h_low, s_low, v_low])
                upper = np.array([h_high, s_high, v_high])
                mask = cv2.inRange(hsv, lower, upper)
            else:
                # Create mask
                lower1, upper1 = np.array(range1[0]), np.array(range1[1])
                mask = cv2.inRange(hsv, lower1, upper1)
                
                if range2:
                    lower2, upper2 = np.array(range2[0]), np.array(range2[1])
                    mask2 = cv2.inRange(hsv, lower2, upper2)
                    mask = cv2.bitwise_or(mask, mask2)
            
            # Clean up mask
            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            
            # Find contours
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            # Track largest object
            if contours:
                largest = max(contours, key=cv2.contourArea)
                
                if cv2.contourArea(largest) > 500:
                    # Get bounding box
                    x, y, w, h = cv2.boundingRect(largest)
                    cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
                    
                    # Get center
                    cx = x + w // 2
                    cy = y + h // 2
                    cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
                    
                    # Add to tracking history
                    center_points.append((cx, cy))
                    if len(center_points) > max_points:
                        center_points.pop(0)
                    
                    # Draw tracking path
                    for i in range(1, len(center_points)):
                        cv2.line(frame, center_points[i-1], center_points[i], 
                                (255, 0, 0), 2)
                    
                    # Display info
                    cv2.putText(frame, f"{color_name} Object", (x, y-10),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                    cv2.putText(frame, f"Center: ({cx}, {cy})", (x, y-30),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            
            # Show frames
            cv2.imshow('Color Tracking', frame)
            cv2.imshow('Mask', mask)
            
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q') or key == 27:
                break
            elif key == ord('r'):
                center_points = []
                print("Tracking reset")
    
    except KeyboardInterrupt:
        print("\nInterrupted")
    finally:
        cap.release()
        cv2.destroyAllWindows()
        print("Camera closed")

if __name__ == "__main__":
    main()
