import cv2
import cv2.aruco as aruco

# Choose dictionary of markers
# DICT_4X4_50 → 50 unique markers, 4x4 grid
aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)

# Pick how many markers you want
num_markers = 5  # make markers 0,1,2,3,4

# Marker size in pixels
marker_size = 500

for marker_id in range(num_markers):
    # Generate marker image
    marker_img = aruco.drawMarker(aruco_dict, marker_id, marker_size)

    # Save to file
    filename = f"aruco_marker_{marker_id}.png"
    cv2.imwrite(filename, marker_img)

    print(f"Saved {filename}")
