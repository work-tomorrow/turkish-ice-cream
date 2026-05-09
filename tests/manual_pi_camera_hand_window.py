import time

import cv2

from hand_tracking import HandDetector, PiCamera2Camera, PiCameraConfig


WINDOW_NAME = "Pi Camera Hand Tracking"


def draw_hand_result(frame, result):
    if not result.detected or result.box is None:
        cv2.putText(
            frame,
            "No hand detected",
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.0,
            (0, 0, 255),
            2,
        )
        return frame

    box = result.box

    x1 = int(box.x1)
    y1 = int(box.y1)
    x2 = int(box.x2)
    y2 = int(box.y2)

    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

    label = f"Hand {result.confidence:.2f}"
    cv2.putText(
        frame,
        label,
        (x1, max(30, y1 - 10)),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (0, 255, 0),
        2,
    )

    center = (int(box.cx), int(box.cy))
    cv2.circle(frame, center, 5, (255, 0, 0), -1)

    return frame


def main():
    detector = HandDetector()

    camera = PiCamera2Camera(
        PiCameraConfig(
            width=640,
            height=480,
            fps=30,
        )
    )

    camera.start()
    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)

    prev_time = time.time()

    try:
        while True:
            frame = camera.read()

            result = detector.detect(frame)
            frame = draw_hand_result(frame, result)

            now = time.time()
            fps = 1.0 / max(now - prev_time, 1e-6)
            prev_time = now

            cv2.putText(
                frame,
                f"FPS: {fps:.1f}",
                (20, frame.shape[0] - 20),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (255, 255, 255),
                2,
            )

            cv2.imshow(WINDOW_NAME, frame)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q") or key == 27:
                break

    finally:
        camera.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()