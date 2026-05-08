# AGENTS.md - `hand_tracking` Agent Guide

This file defines how future agents should work inside
`packages/hand_tracking/`. Treat it as the local operating manual for this
package.

## Mission

`hand_tracking` owns hand-perception concerns.

This package is responsible for:

- reading frames from the Raspberry Pi camera
- converting camera frames into OpenCV BGR frames
- wrapping MediaPipe Hand Landmarker behind one simple Python API
- providing simple JPEG / MJPEG streaming helpers for upper layers

Upper layers should not need to know Picamera2 setup, MediaPipe setup, model
paths, landmarks, OpenCV color conversion, or bounding-box calculation.

The main hand detection usage should remain simple:

```python
from hand_tracking import HandDetector

detector = HandDetector()
result = detector.detect(frame)
```

The main camera usage should also remain simple:

```python
from hand_tracking import PiCamera2Camera

camera = PiCamera2Camera()
camera.start()

try:
    frame = camera.read()
finally:
    camera.stop()
```

`camera.read()` must return an OpenCV image frame in BGR format.

## Public API

The main public hand detection entry point is `HandDetector`.

Expected hand detection usage:

```python
from hand_tracking import HandDetector

detector = HandDetector()
hand = detector.detect(frame)

if hand.detected and hand.box is not None:
    x = hand.box.cx
    y = hand.box.cy
    area = hand.box.area
```

`frame` must be an OpenCV image frame in BGR format.

`detect(frame)` should return a `HandDetectionResult` with:

- `detected`: whether a hand was found
- `box`: bounding box around the detected hand, or `None`
- `confidence`: detection-related score

The bounding box should expose:

- `x1`, `y1`, `x2`, `y2`
- `cx`, `cy`
- `width`, `height`
- `area`

The main public camera entry point is `PiCamera2Camera`.

Expected camera usage:

```python
from hand_tracking import PiCamera2Camera

camera = PiCamera2Camera()
camera.start()

try:
    frame = camera.read()
finally:
    camera.stop()
```

`PiCamera2Camera.read()` should return one OpenCV BGR frame.

Expected combined usage:

```python
from hand_tracking import HandDetector, PiCamera2Camera

detector = HandDetector()
camera = PiCamera2Camera()
camera.start()

try:
    while True:
        frame = camera.read()
        hand = detector.detect(frame)

        if hand.detected and hand.box is not None:
            print(hand.box.cx, hand.box.cy, hand.box.area)
finally:
    camera.stop()
```

Expected streaming usage:

```python
from hand_tracking import PiCamera2Camera, stream_camera_mjpeg

camera = PiCamera2Camera()

for chunk in stream_camera_mjpeg(camera):
    ...
```

`stream_camera_mjpeg(camera)` yields MJPEG chunks. Upper layers may wrap these
chunks in an HTTP streaming response, WebSocket handler, or other transport.

## Hard Rules

- Keep `HandDetector` as the main hand detection entry point.
- Keep `PiCamera2Camera` as the main Raspberry Pi camera entry point.
- Pi Camera access is allowed in this package, but keep it isolated in
  `camera.py`.
- JPEG / MJPEG streaming helpers are allowed in this package, but keep them
  isolated in `stream.py`.
- Do not put FastAPI, motor control, robot behavior, or session state in this
  package.
- Do not make `hand_tracking` depend on backend API framework details.
- Do not make upper layers import or configure MediaPipe directly.
- Do not make upper layers import or configure Picamera2 directly unless there
  is a clear reason.
- Do not make upper layers handle hand landmarks directly unless explicitly
  requested.
- Keep the current detector contract:
  `detect(frame) -> HandDetectionResult`.
- Camera wrappers must return OpenCV BGR frames.
- Keep the default model path inside this package:
  `packages/hand_tracking/models/hand_landmarker.task`.
- Do not require a webcam or Pi Camera for package-level tests.
- Keep tests runnable in WSL or a normal development environment using image or
  video files.
- Do not commit temporary test images unless they are intentionally added as
  small test assets.
- Remove generated `__pycache__` directories before finishing.

## Current Architecture

Keep these dependency directions:

```text
upper layer
  -> PiCamera2Camera
      -> Picamera2
          -> Raspberry Pi camera
```

```text
upper layer
  -> stream_camera_mjpeg(camera)
      -> camera.read()
      -> JPEG / MJPEG bytes
```

```text
upper layer
  -> HandDetector
      -> MediaPipe Hand Landmarker
          -> hand_landmarker.task
```

Responsibilities:

- `PiCamera2Camera`: configure Picamera2, start the Raspberry Pi camera, capture
  frames, and convert captured frames into OpenCV BGR format.
- `stream_camera_jpeg`: read frames from a camera and yield JPEG bytes.
- `stream_camera_mjpeg`: read frames from a camera and yield MJPEG chunks.
- `HandDetector`: load the MediaPipe model, accept OpenCV BGR frames, run
  detection, and convert MediaPipe output into project-friendly results.
- `Box`: represent the detected hand bounding box in pixel coordinates.
- `HandDetectionResult`: represent whether a hand was detected and the result
  data needed by upper layers.
- `models/`: store the local MediaPipe `.task` model file.

The intended runtime pipeline is:

```text
PiCamera2Camera.read()
  -> OpenCV BGR frame
  -> HandDetector.detect(frame)
  -> HandDetectionResult
```

For frontend camera preview, the intended pipeline is:

```text
PiCamera2Camera.read()
  -> OpenCV BGR frame
  -> JPEG encode
  -> MJPEG chunks
  -> upper-layer API / frontend transport
```

## Camera File

Expected camera abstraction file:

```text
packages/hand_tracking/camera.py
```

Expected camera classes and types:

```python
Camera
PiCameraConfig
PiCamera2Camera
```

`Camera` should define the common camera interface:

```python
camera.start()
frame = camera.read()
camera.stop()
```

`PiCamera2Camera` should use Picamera2 internally.

Picamera2 may return frames in RGB format when configured with `RGB888`.
Since `HandDetector.detect(frame)` expects OpenCV BGR frames, `PiCamera2Camera`
must convert RGB frames to BGR before returning them.

Do not use `cv2.VideoCapture(0)` as the main Pi Camera implementation.
`cv2.VideoCapture(0)` may still be useful for optional USB webcam or local
development support, but Raspberry Pi CSI camera support should prioritize
Picamera2.

## Stream File

Expected stream helper file:

```text
packages/hand_tracking/stream.py
```

Expected stream helpers:

```python
encode_frame_to_jpeg(frame, jpeg_quality=80)
stream_camera_jpeg(camera, jpeg_quality=80)
stream_camera_mjpeg(camera, jpeg_quality=80)
```

`stream_camera_jpeg(camera)` should yield raw JPEG bytes.

`stream_camera_mjpeg(camera)` should yield chunks compatible with:

```text
multipart/x-mixed-replace; boundary=frame
```

Do not import FastAPI in `stream.py`.

Upper layers are responsible for choosing how to expose the stream to the web
frontend.

## Model File

Expected model location:

```text
packages/hand_tracking/models/hand_landmarker.task
```

To re-download the model:

```sh
mkdir -p packages/hand_tracking/models

curl -L \
  -o packages/hand_tracking/models/hand_landmarker.task \
  https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/float16/latest/hand_landmarker.task
```

If this model file is missing, `HandDetector()` should fail clearly with
`FileNotFoundError`.

## Editing Guidelines

- Prefer small, local changes.
- Keep the public API stable unless the caller requirements change.
- Keep MediaPipe-specific details inside `detector.py`.
- Keep Picamera2-specific details inside `camera.py`.
- Keep stream encoding details inside `stream.py`.
- Keep return types simple and explicit.
- Camera read methods should return OpenCV BGR frames, not RGB frames.
- If adding multi-hand support, avoid breaking the existing single-hand API
  without updating upper-layer callers.
- If changing confidence semantics, document whether it means handedness score,
  detection score, or another score.
- Keep comments short and useful.

## Common Pitfalls

- Do not assume WSL can access the host webcam.
- Do not assume CI can access a real camera.
- Do not assume Picamera2 is installed outside Raspberry Pi.
- Do not use `cv2.VideoCapture(0)` as the default Pi Camera path.
- Do not return RGB frames from `PiCamera2Camera.read()`.
- Do not treat MediaPipe landmarks as pixel coordinates directly; `x` and `y`
  are normalized and must be multiplied by image width and height.
- Do not expose raw MediaPipe result objects to upper layers.
- Do not expose raw Picamera2 objects to upper layers unless explicitly needed.
- Do not make robot behavior depend on MediaPipe internals.
- Do not make web API code depend on MediaPipe or Picamera2 internals.
- Do not confuse `confidence` with YOLO-style bounding-box confidence unless the
  implementation actually provides that semantics.
- Do not move the model file outside the package without updating the default
  path.
