#!/usr/bin/env python3
"""
Video recording from RealSense camera
Records RGB and/or IR streams to video files
"""

import cv2
from datetime import datetime

def main():
    print("=" * 60)
    print("Video Recorder - RealSense Camera")
    print("=" * 60)
    
    # Choose camera(s)
    print("\nSelect camera to record:")
    print("  1 - RGB only")
    print("  2 - IR only")
    print("  3 - Both RGB + IR")
    
    choice = input("\nChoice (1-3): ").strip()
    
    record_rgb = choice in ['1', '3']
    record_ir = choice in ['2', '3']
    
    caps = []
    writers = []
    filenames = []
    
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    
    # Setup RGB
    if record_rgb:
        cap_rgb = cv2.VideoCapture(4)
        if cap_rgb.isOpened():
            caps.append(('RGB', cap_rgb))
            fourcc = cv2.VideoWriter_fourcc(*'XVID')
            filename = f"/home/arash/cogrob/video_rgb_{timestamp}.avi"
            writer = cv2.VideoWriter(filename, fourcc, 30.0, (640, 480))
            writers.append(writer)
            filenames.append(filename)
            print(f"RGB camera ready: {filename}")
        else:
            print("Cannot open RGB camera")
    
    # Setup IR
    if record_ir:
        cap_ir = cv2.VideoCapture(2)
        if cap_ir.isOpened():
            caps.append(('IR', cap_ir))
            fourcc = cv2.VideoWriter_fourcc(*'XVID')
            filename = f"/home/arash/cogrob/video_ir_{timestamp}.avi"
            writer = cv2.VideoWriter(filename, fourcc, 30.0, (640, 480))
            writers.append(writer)
            filenames.append(filename)
            print(f"IR camera ready: {filename}")
        else:
            print("Cannot open IR camera")
    
    if not caps:
        print("❌ No cameras available")
        return
    
    print("\nControls:")
    print("  SPACE - Start/Stop recording")
    print("  'q' - Quit\n")
    
    recording = False
    frame_count = 0
    
    try:
        while True:
            frames = []
            
            # Capture from all cameras
            for name, cap in caps:
                ret, frame = cap.read()
                if ret:
                    # Add recording indicator
                    if recording:
                        cv2.circle(frame, (30, 30), 15, (0, 0, 255), -1)
                        cv2.putText(frame, "REC", (50, 40),
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                    
                    cv2.putText(frame, f"{name} - Frame: {frame_count}", (10, 470),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                    
                    frames.append(frame)
                else:
                    print(f"Failed to capture from {name}")
                    break
            
            if len(frames) != len(caps):
                break
            
            # Write frames if recording
            if recording:
                for i, writer in enumerate(writers):
                    writer.write(frames[i])
                frame_count += 1
            
            # Display
            if len(frames) == 1:
                cv2.imshow('Camera', frames[0])
            else:
                import numpy as np
                combined = np.hstack(frames)
                cv2.imshow('Cameras', combined)
            
            key = cv2.waitKey(1) & 0xFF
            
            if key == ord('q') or key == 27:
                break
            elif key == ord(' '):  # Space bar
                recording = not recording
                if recording:
                    print(f"\nRecording started...")
                    frame_count = 0
                else:
                    print(f"Recording stopped. Frames: {frame_count}")
    
    except KeyboardInterrupt:
        print("\nInterrupted")
    finally:
        # Cleanup
        for _, cap in caps:
            cap.release()
        for writer in writers:
            writer.release()
        cv2.destroyAllWindows()
        
        print("\n" + "=" * 60)
        print("Recording complete!")
        print("=" * 60)
        for filename in filenames:
            print(f"  {filename}")

if __name__ == "__main__":
    main()
