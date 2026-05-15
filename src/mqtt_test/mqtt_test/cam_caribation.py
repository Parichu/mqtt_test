import cv2
import numpy as np
import json
import time

# --- STEP 1: YOUR TEST CLICKED VALUES ---
image_points = np.array(
    [
        [262, 432],  # Your Click 1
        [972, 322],  # Your Click 2
        [1278, 348],  # Your Click 3
        [1278, 718],  # Your Click 4
    ],
    dtype=np.float32,
)

# --- STEP 2: SIMULATED MAP (2m x 2m square) ---
map_points = np.array(
    [
        [0, 2],  # Far Left
        [2, 2],  # Far Right
        [0, 0],  # Near Left
        [2, 0],  # Near Right
    ],
    dtype=np.float32,
)


# Calculate the Matrix
H, _ = cv2.findHomography(image_points, map_points)


def simulate_click(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        # Convert the pixel click to a Matrix-compatible format
        px_point = np.array([[[x, y]]], dtype=np.float32)

        # TRANSFORMATION MATH
        real_world = cv2.perspectiveTransform(px_point, H)

        target_x = real_world[0][0][0]
        target_y = real_world[0][0][1]

        print(
            f"PIXEL: ({x}, {y}) ---> ROBOT WILL GO TO: {target_x:.2f}m, {target_y:.2f}m"
        )


# --- STEP 3: VISUAL TEST ---
cap = cv2.VideoCapture(0)
cv2.namedWindow("Simulation")
cv2.setMouseCallback("Simulation", simulate_click)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    cv2.putText(
        frame,
        "CLICK ANYWHERE TO SEE ROBOT METERS",
        (20, 50),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (255, 255, 0),
        2,
    )
    cv2.imshow("Simulation", frame)

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()
