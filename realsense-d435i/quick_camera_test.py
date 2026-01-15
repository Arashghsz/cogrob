#!/usr/bin/env python3
"""Quick RGB camera view from RealSense D435i"""

import cv2

# Use video device 4 for RGB
cap = cv2.VideoCapture(4)

if not cap.isOpened():
    print("Cannot open camera on /dev/video4")
    exit(1)

print("RealSense RGB Camera - Press 'q' to quit")

while True:
    ret, frame = cap.read()
    if not ret:
        break
    
    cv2.imshow('RealSense RGB', frame)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
