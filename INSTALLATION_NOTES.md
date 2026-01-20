# Franky Installation on Jetson AGX Orin - Status and Issues

## Current Status
✅ Real-time kernel installed (`PREEMPT_RT`)  
✅ User added to realtime group  
✅ libfranka 0.18.0 built and installed successfully to `/usr/local/lib`  
✅ franky built and installed in virtual environment  
❌ **Segmentation fault when creating Affine objects**

## The Problem
Franky experiences segmentation faults on ARM64 Jetson platform when creating Eigen-based objects (like Affine). This is an ABI (Application Binary Interface) incompatibility issue, likely caused by:
- Mismatched Eigen versions between compilation and runtime
- pybind11 version incompatibilities  
- ARM64-specific compilation issues

## What Was Tried
1. ✅ Removed ROS2 libfranka (had pinocchio dependencies causing conflicts)
2. ✅ Built standalone libfranka 0.18.0 from source
3. ✅ Built franky from source against clean libfranka
4. ❌ Still segfaults on Eigen object creation

## Recommended Solutions

### Option 1: Use libfranka Directly (Without Franky)
Since libfranka is working, you can use it directly from C++ or Python bindings:

```bash
# libfranka is installed at /usr/local
# Headers: /usr/local/include/franka/
# Library: /usr/local/lib/libfranka.so.0.18
```

Example C++ program with libfranka:
```cpp
#include <franka/robot.h>
#include <franka/exception.h>

int main() {
  franka::Robot robot("172.16.0.2");
  // Your control code here
}
```

Compile with:
```bash
g++ -std=c++14 my_program.cpp -o my_program -lfranka -I/usr/local/include -L/usr/local/lib
```

### Option 2: Use frankx (Alternative to franky)
[frankx](https://github.com/pantor/frankx) is an alternative library that franky was forked from.

### Option 3: Use franka_ros2
Since you have ROS2 Humble installed, use the official `franka_ros2` packages:
```bash
sudo apt install ros-humble-franka-ros2
```

### Option 4: Docker with x86_64 Emulation
Run franky in Docker with x86_64 emulation (slower but might work):
```bash
docker run --platform linux/amd64 -it --network host ubuntu:22.04
# Then install franky normally
```

## Files Created
- `/home/arash/franky/run_franky.sh` - Wrapper script with correct library paths
- `LD_LIBRARY_PATH` must include `/usr/local/lib` for libfranka

## Environment Setup
Add to your `~/.zshrc` or `~/.bashrc`:
```bash
export LD_LIBRARY_PATH="/usr/local/lib:$LD_LIBRARY_PATH"
```

## Contact/Support
- Franky GitHub: https://github.com/TimSchneider42/franky/issues
- Consider reporting this ARM64/Jetson issue to the franky maintainers
