import cv2

# 1. Setup Camera
# CAP_V4L2 is the standard Video For Linux driver
cap = cv2.VideoCapture(2, cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1980)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

# State variable to store the clicked point
point = None


def click_event(event, x, y, flags, param):
    global point
    # Listen for Left Button Double Click
    if event == cv2.EVENT_LBUTTONDBLCLK:
        point = (x, y)
        print(f"Clicked Coordinates: X: {x}, Y: {y}")


# 2. Create the window FIRST so we can attach a callback to it
cv2.namedWindow("Robot Eye")
cv2.setMouseCallback("Robot Eye", click_event)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # 3. If a point exists, draw it on the current live frame
    if point is not None:
        # Draw a solid red circle
        cv2.circle(frame, point, 7, (0, 0, 255), -1)
        # Optional: Display coordinates next to the dot
        cv2.putText(
            frame,
            str(point),
            (point[0] + 10, point[1] - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 0, 255),
            1,
        )

    cv2.imshow("Robot Eye", frame)

    key = cv2.waitKey(1) & 0xFF
    if key == ord("q"):
        break
    elif key == ord("c"):  # Press 'c' to clear the point
        point = None

cap.release()
cv2.destroyAllWindows()
