import cv2
import numpy as np

# ==============================================================================
# 1. CALIBRATION DATA
# Paste the values you got from the calibration step here.
# ==============================================================================

# Pixel coordinates from the camera {u, v}
image_points = np.array(
    [
        [262, 432],
        [972, 322],
        [1278, 348],
        [1278, 718],
    ],
    dtype=np.float32,
)

# Real-world map coordinates in meters {x, y}
map_points = np.array(
    [
        [0.0, 2.0],
        [2.0, 2.0],
        [0.0, 0.0],
        [2.0, 0.0],
    ],
    dtype=np.float32,
)

# Calculate the Homography Matrix (H)
H, _ = cv2.findHomography(image_points, map_points)

# ==============================================================================
# 2. TRANSFORMATION FUNCTION
# ==============================================================================


def cam_px_to_map(u: int, v: int) -> tuple:
    """Converts a Camera pixel (u, v) to Map coordinates (x, y) in meters."""
    # Create a 3D array required by cv2.perspectiveTransform
    pt = np.array([[[float(u), float(v)]]], dtype=np.float32)

    # Apply the homography matrix
    transformed_pt = cv2.perspectiveTransform(pt, H)

    # Extract the x and y values
    x = float(transformed_pt[0][0][0])
    y = float(transformed_pt[0][0][1])

    return x, y


# ==============================================================================
# 3. CAMERA AND MOUSE CLICK LOGIC
# ==============================================================================


def on_mouse_click(event, u, v, flags, param):
    """Callback function triggered when the mouse is clicked on the camera feed."""
    if event == cv2.EVENT_LBUTTONDOWN:
        # Transform the clicked {u, v} to {x, y}
        target_x, target_y = cam_px_to_map(u, v)

        # Print the result to the terminal
        print("-" * 40)
        print(f"Clicked Camera Pixel {{u, v}}: ({u}, {v})")
        print(
            f"Transformed Map Target {{x, y}}: ({target_x:.3f}, {target_y:.3f}) meters"
        )
        print("-" * 40)


def main():
    # Open the external camera (device 2, change if necessary)
    cap = cv2.VideoCapture(2, cv2.CAP_V4L2)

    # Set desired resolution
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

    if not cap.isOpened():
        print("Error: Cannot open camera.")
        return

    # Create a window and attach the mouse callback
    window_name = "Camera Feed - Click to Transform"
    cv2.namedWindow(window_name)
    cv2.setMouseCallback(window_name, on_mouse_click)

    print("Camera opened successfully.")
    print(
        "Click anywhere on the video feed to see the {u, v} to {x, y} transformation."
    )
    print("Press 'q' to quit.")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame.")
            break

        # Draw visual markers for the calibration points (optional, for reference)
        for pt in image_points.astype(int):
            cv2.drawMarker(frame, tuple(pt), (0, 220, 255), cv2.MARKER_CROSS, 20, 2)

        cv2.putText(
            frame,
            "Click to get {u,v} -> {x,y} | 'q' to quit",
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            (0, 255, 0),
            2,
        )

        # Show the video feed
        cv2.imshow(window_name, frame)

        # Quit if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
