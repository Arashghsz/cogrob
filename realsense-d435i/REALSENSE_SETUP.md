# RealSense D435i on Jetson AGX Orin

## Camera Status

✅ **Intel RealSense D435i**: Working with OpenCV
- RGB Stream: `/dev/video4` (640x480 @ 30fps)
- IR Stream: `/dev/video2` (640x480 @ 30fps)

## Why OpenCV Only?

The pyrealsense2 SDK's depth pipeline hangs on Jetson ARM64, even with the Jetson-specific SDK. This is a known hardware limitation that would require kernel patches.

**Solution**: All scripts use OpenCV's V4L2 interface for camera access - no SDK needed, everything works reliably.

## Available Scripts

| Script | Purpose | Controls |
|--------|---------|----------|
| `quick_camera_test.py` | Simple RGB viewer | 'q' to quit |
| `motion_detection.py` | Motion detection with sensitivity control | 's' to save, '+/-' adjust sensitivity, 'q' to quit |
| `color_tracking.py` | Track colored objects with path history | 'q' to quit, 'r' to reset tracking |
| `edge_detection.py` | Multiple edge detection methods (Canny/Sobel/Laplacian) | '1-4' to switch methods, 'q' to quit |
| `video_recorder.py` | Record RGB or IR to video file | SPACE to start/stop, 'q' to quit |

## Quick Start Examples

### 1. View Live RGB Camera
```bash
python quick_camera_test.py
```

### 2. Detect Motion (Security)
```bash
python motion_detection.py
# Press 's' when motion appears to save snapshot
# Use +/- keys to adjust sensitivity
```

### 3. Track Colored Objects (Robotics)
```bash
python color_tracking.py
# Select a color to track (red, green, blue, yellow, or custom)
# Shows real-time tracking path on screen
```

### 4. Edge Detection (Navigation/Path Planning)
```bash
python edge_detection.py
# Press 1-4 to try different edge detection algorithms
# Useful for obstacle detection
```

### 5. Record Video for Analysis
```bash
python video_recorder.py
# Choose RGB, IR, or both
# Press SPACE to start/stop recording
# Saves to AVI files
```

## Installation & Setup

### Requirements
```bash
pip install opencv-python numpy
```

### Camera Access
- Connect camera to **USB 3.0 port** (blue) on Jetson
- Camera will be auto-detected at `/dev/video4` (RGB) and `/dev/video2` (IR)

### Verify Camera is Working
```bash
# Check device
v4l2-ctl --list-devices

# Simple test
python quick_camera_test.py
```

## Troubleshooting

**"Cannot open camera" error?**
- Verify USB 3.0 connection (blue port)
- Check device: `v4l2-ctl --list-devices`
- Try: `lsusb | grep Intel`

**No video window appears?**
- Ensure X11 forwarding enabled (if using SSH)
- Check: `echo $DISPLAY` (should not be empty)

**Low frame rate or stuttering?**
- Move camera to dedicated USB 3.0 port
- Close other applications using camera
- Reduce monitor resolution if needed

## Code Example: Basic Frame Capture

```python
import cv2
import numpy as np

# Open RGB camera
cap = cv2.VideoCapture(4)  # /dev/video4 = RGB

while True:
    ret, frame = cap.read()
    
    if not ret:
        print("Failed to read frame")
        break
    
    # Your processing here
    cv2.imshow('RealSense RGB', frame)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
```

## Performance Notes

- All scripts run in real-time at ~30fps
- CPU usage varies by script (motion detection uses more)
- IR stream has lower brightness but works well for tracking
- Video recording saves at camera FPS (~30fps)
