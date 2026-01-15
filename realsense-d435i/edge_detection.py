#!/usr/bin/env python3
"""
Edge detection with multiple algorithms
Useful for robot vision and navigation
"""

import cv2
import numpy as np

def main():
    print("=" * 60)
    print("Edge Detection - RealSense RGB Camera")
    print("=" * 60)
    
    cap = cv2.VideoCapture(4)
    
    if not cap.isOpened():
        print("❌ Cannot open camera")
        return
    
    print("\nCamera ready")
    print("\nControls:")
    print("  '1' - Canny edge detection")
    print("  '2' - Sobel edge detection")
    print("  '3' - Laplacian edge detection")
    print("  '4' - Show all methods")
    print("  'q' - Quit\n")
    
    mode = 1
    canny_low = 50
    canny_high = 150
    
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                break
            
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            blurred = cv2.GaussianBlur(gray, (5, 5), 0)
            
            # Canny
            canny = cv2.Canny(blurred, canny_low, canny_high)
            canny_color = cv2.cvtColor(canny, cv2.COLOR_GRAY2BGR)
            
            # Sobel
            sobelx = cv2.Sobel(blurred, cv2.CV_64F, 1, 0, ksize=3)
            sobely = cv2.Sobel(blurred, cv2.CV_64F, 0, 1, ksize=3)
            sobel = np.sqrt(sobelx**2 + sobely**2)
            sobel = np.uint8(sobel / sobel.max() * 255)
            sobel_color = cv2.cvtColor(sobel, cv2.COLOR_GRAY2BGR)
            
            # Laplacian
            laplacian = cv2.Laplacian(blurred, cv2.CV_64F)
            laplacian = np.uint8(np.absolute(laplacian))
            laplacian_color = cv2.cvtColor(laplacian, cv2.COLOR_GRAY2BGR)
            
            # Display based on mode
            if mode == 1:
                cv2.putText(canny_color, "Canny Edge Detection", (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                display = np.hstack((frame, canny_color))
            elif mode == 2:
                cv2.putText(sobel_color, "Sobel Edge Detection", (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                display = np.hstack((frame, sobel_color))
            elif mode == 3:
                cv2.putText(laplacian_color, "Laplacian Edge Detection", (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                display = np.hstack((frame, laplacian_color))
            else:  # mode 4 - show all
                top = np.hstack((frame, canny_color))
                bottom = np.hstack((sobel_color, laplacian_color))
                
                # Add labels
                cv2.putText(frame, "Original", (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.putText(canny_color, "Canny", (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.putText(sobel_color, "Sobel", (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.putText(laplacian_color, "Laplacian", (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                
                display = np.vstack((top, bottom))
                display = cv2.resize(display, (1280, 960))
            
            cv2.imshow('Edge Detection', display)
            
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q') or key == 27:
                break
            elif key == ord('1'):
                mode = 1
                print("Mode: Canny")
            elif key == ord('2'):
                mode = 2
                print("Mode: Sobel")
            elif key == ord('3'):
                mode = 3
                print("Mode: Laplacian")
            elif key == ord('4'):
                mode = 4
                print("Mode: All methods")
            elif key == ord('+'):
                canny_high = min(255, canny_high + 10)
                print(f"Canny high: {canny_high}")
            elif key == ord('-'):
                canny_high = max(canny_low + 10, canny_high - 10)
                print(f"Canny high: {canny_high}")
    
    except KeyboardInterrupt:
        print("\nInterrupted")
    finally:
        cap.release()
        cv2.destroyAllWindows()
        print("Camera closed")

if __name__ == "__main__":
    main()
