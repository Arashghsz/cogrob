# Whisper ROS Detailed Test Report

## Overview

This report documents a local test of `whisper_ros` on ROS 2 Jazzy.
The project provides speech-to-text (STT) using Whisper and voice activity
detection (VAD) using Silero.

## Environment

- Operating system: Linux
- ROS 2: Jazzy
- Workspace: `/home/arash/whisper_ros`
- CPU execution was used.
- Docker was available but the Docker socket was not accessible.

## Setup

The external `audio_common` dependency was imported from
`dependencies.repos`.

The workspace was built with the ROS system Python environment because the
user Python interpreter did not contain required ROS modules such as
`catkin_pkg` and `em`.

```bash
cd /home/arash/whisper_ros
vcs import . < dependencies.repos

env -u AMENT_PREFIX_PATH \
    -u CMAKE_PREFIX_PATH \
    -u COLCON_PREFIX_PATH \
    -u PYTHONPATH \
    -u PYTHONHOME \
    PATH=/usr/bin:/bin:/opt/ros/jazzy/bin \
    bash -c '
      source /opt/ros/jazzy/setup.bash
      colcon build --merge-install
    '
```

Build result:

```text
Summary: 9 packages finished
```

## Start Whisper with microphone input

```bash
cd /home/arash/whisper_ros
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch whisper_bringup whisper.launch.py
```

The first launch downloads the configured models:

- Whisper model: approximately 452 MB
- Silero VAD model: approximately 2 MB

The models are cached after the first download.

## Start without microphone input

This command tests node startup and model loading without using a microphone:

```bash
ros2 launch whisper_bringup whisper.launch.py \
  launch_audio_capturer:=False \
  silero_vad_enabled:=False
```

Observed successful startup messages included:

```text
WhisperServer node started
[whisper_node] Configuring...
[whisper_node] Activated
Processing, 4 threads, lang = en, task = transcribe
SileroVAD Iterator started
```

## Check running nodes

```bash
ros2 node list
```

With microphone capture enabled, the expected nodes are:

```text
/audio/capturer_node
/whisper/silero_vad_node
/whisper/whisper_node
```

If `/audio/capturer_node` is missing, the microphone is not being used.

## Request a transcription

Keep the launch command running in one terminal. In a second terminal:

```bash
source /opt/ros/jazzy/setup.bash
source /home/arash/whisper_ros/install/setup.bash
ros2 action send_goal /whisper/listen whisper_msgs/action/STT "{}"
```

The empty `{}` is intentional. The action starts listening; it does not
accept text as its goal.

After the goal is accepted, speak clearly and pause at the end of the
sentence. The result should contain:

```text
transcription:
  text: "hello this is a test"
  audio_time: ...
  transcription_time: ...
```

## Text-to-speech

`whisper_ros` does not provide text-to-speech. It only converts speech into
text.

The local system command `spd-say` can be used separately:

```bash
spd-say "Hello, this is a text to speech test"
```

The default system voice may sound robotic. A neural TTS engine such as
Piper is a better option for natural-sounding speech.

## CPU and GPU behavior

The tested build ran on the CPU and reported:

```text
whisper_backend_init_gpu: no GPU found
```

The CPU build works, but transcription is slower than a CUDA build.
For NVIDIA GPU support, rebuild with:

```bash
colcon build --merge-install \
  --cmake-args -DGGML_CUDA=ON -DONNX_GPU=ON
```

Then launch with:

```bash
ros2 launch whisper_bringup whisper.launch.py \
  silero_vad_use_cuda:=True
```

## Pros

- Runs locally without a cloud API.
- Integrates with ROS 2 actions, topics, and nodes.
- Includes Whisper speech recognition.
- Includes Silero voice activity detection.
- Supports CPU execution.
- Supports optional CUDA acceleration.
- Provides a reusable STT action interface.

## Cons

- The initial model download is large.
- CPU inference is slower than GPU inference.
- Microphone and audio permissions may require separate configuration.
- The default configuration uses a large model and can require substantial
  memory.
- The project is speech-to-text only.
- Text-to-speech needs a separate package or command.
- `spd-say` may produce robotic-sounding speech.

## Final status

The workspace built successfully, the Whisper and Silero nodes started
successfully, the model loaded, and the `/whisper/listen` action accepted
transcription requests.
