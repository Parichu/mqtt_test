import cv2
import sys
import os

# Metadata from mapuse28.yaml
RESOLUTION = 0.03
ORIGIN_X = -3.76
ORIGIN_Y = -3.22
HEIGHT = 288

def on_mouse_click(event, u, v, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        # Convert pixel to map coordinates
        x = ORIGIN_X + (u * RESOLUTION)
        y = ORIGIN_Y + ((HEIGHT - 1 - v) * RESOLUTION)
        
        print("\n" + "="*50)
        print(f"📍 GOAL POINT SELECTED")
        print(f"Pixel: u={u}, v={v}")
        print(f"Map Coordinates: x={x:.3f}, y={y:.3f}")
        print("="*50)
        
        print("\n▶ To send the robot to this point, copy and paste this Action command in a new terminal:")
        print(f"ros2 action send /navigate_to_pose nav2_msgs/action/NavigateToPose \"{{pose: {{header: {{frame_id: 'map'}}, pose: {{position: {{x: {x:.3f}, y: {y:.3f}, z: 0.0}}, orientation: {{w: 1.0}}}}}}}}\"")

def main():
    map_path = 'src/mqtt_test/config/mapuse28.pgm'
    if not os.path.exists(map_path):
        print(f"Error: Map file not found at {map_path}")
        print("Please run this script from the root of your workspace: /home/parichu/ros2_ws/mqtt_test")
        sys.exit(1)

    # Load your map image
    img = cv2.imread(map_path)

    if img is None:
        print("Could not load image using OpenCV.")
        sys.exit(1)
        
    print("Map loaded! Click anywhere on the map to get the coordinate command. Press 'q' to quit.")
    cv2.namedWindow('Click Map for Target Point')
    cv2.setMouseCallback('Click Map for Target Point', on_mouse_click)
    
    while True:
        cv2.imshow('Click Map for Target Point', img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
