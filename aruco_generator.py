import cv2
import cv2.aruco as aruco
import os

# -----------------------------
# CONFIG
# -----------------------------
MARKER_SIZE = 700
DICT_TYPE = aruco.DICT_4X4_50
NUM_MARKERS = 6        # you said you'll likely use multiple markers
OUTPUT_DIR = "aruco_markers"

# -----------------------------
# CREATE OUTPUT FOLDER
# -----------------------------
os.makedirs(OUTPUT_DIR, exist_ok=True)

# -----------------------------
# LOAD ARUCO DICTIONARY
# -----------------------------
aruco_dict = aruco.getPredefinedDictionary(DICT_TYPE)

# -----------------------------
# GENERATE MARKERS
# -----------------------------
for marker_id in range(NUM_MARKERS):
    marker_img = aruco.generateImageMarker(
        aruco_dict,
        marker_id,
        MARKER_SIZE
    )

    filename = os.path.join(OUTPUT_DIR, f"aruco_{marker_id}.png")
    cv2.imwrite(filename, marker_img)

    print(f"Saved: {filename}")

print("\nDone generating ArUco markers.")