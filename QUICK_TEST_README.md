# Whisper ROS Quick Test Report

## What I tested

- Built the ROS 2 Jazzy workspace with `audio_common`.
- Started Whisper and Silero VAD successfully.
- Confirmed the Whisper model loads on CPU.
- Sent a `/whisper/listen` action goal.
- Confirmed that `spd-say` provides text-to-speech separately.

## Commands

Build:

```bash
cd /home/arash/whisper_ros
vcs import . < dependencies.repos
source /opt/ros/jazzy/setup.bash
colcon build --merge-install
source install/setup.bash
```

Start speech-to-text with microphone:

```bash
ros2 launch whisper_bringup whisper.launch.py
```

Request a transcription in another terminal:

```bash
ros2 action send_goal /whisper/listen whisper_msgs/action/STT "{}"
```

Test text-to-speech separately:

```bash
spd-say "Hello, this is a text to speech test"
```

Check running nodes:

```bash
ros2 node list
```

## Pros

- Runs locally with no cloud service.
- Supports ROS 2 integration.
- Includes Whisper speech recognition and Silero voice activity detection.
- Supports CPU execution and optional GPU builds.

## Cons

- Initial model download is large.
- CPU inference is slower than CUDA.
- `whisper_ros` is speech-to-text only; text-to-speech requires a separate tool.
- The default `spd-say` voice may sound robotic.
