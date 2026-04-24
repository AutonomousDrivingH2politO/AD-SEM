import cv2
import numpy as np
import pyzed.sl as sl  # ZED SDK
import os


def run_calibration(marker_size_meters=0.5):
    zed = sl.Camera()
    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.HD1080
    init_params.coordinate_units = sl.UNIT.METER

    if zed.open(init_params) != sl.ERROR_CODE.SUCCESS:
        print("Failed to open the camera")
        return

    # Capturing a frame
    image_zed = sl.Mat()
    if zed.grab() == sl.ERROR_CODE.SUCCESS:
        zed.retrieve_image(image_zed, sl.VIEW.LEFT)
        frame = image_zed.get_data()
        frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
    else:
        print("couldn't grab frame")
        return

    # 3. Detect ArUco Marker
    # We use DICT_4X4_50.
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    parameters = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

    corners, ids, rejected = detector.detectMarkers(frame)

    if ids is not None:
        print(f"Marker detected! ID: {ids[0]}")
        # corners[0][0] contains: [top-left, top-right, bottom-right, bottom-left]
        src_pts = corners[0][0].astype(np.float32)

        # 4. Define Destination (The "Perfect Rectangle" in BEV)
        # We define a scale: 1 meter = 200 pixels
        # but bev width is 1280 pixels, at 200px, total width of the ground camera can see 1280/200 = 6.4m
        # This means lane is wider than our image, might cause lane lines will be cropped out of the screen

        # In your old manual method (utils_path.py), 10.0 meters was at row 448 and 7.5 meters was at row 530. That is a
        # difference of 82 pixels for 2.5 meters, which means your old scale was exactly 32.8 pixels per meter.
        px_per_meter = 200
        marker_px = marker_size_meters * px_per_meter

        # We want the marker to appear centered at the bottom of our 800x800 BEV image
        # bev_size = 800
        bev_w, bev_h = 1280, 720
        center_x = bev_w // 2
        bottom_y = bev_h - 50  # Put the marker near the bottom of the 720p image

        dst_pts = np.float32([
            [center_x - marker_px/2, bottom_y - marker_px],  # Top-Left
            [center_x + marker_px/2, bottom_y - marker_px],  # Top-Right
            [center_x + marker_px/2, bottom_y],              # Bottom-Right
            [center_x - marker_px/2, bottom_y]               # Bottom-Left
        ])

        # calculate and save the matrix
        M = cv2.getPerspectiveTransform(src_pts, dst_pts)
        np.save("bev_matrix.npy", M)

        # preview
        warped = cv2.warpPerspective(frame, M, (bev_w, bev_h))
        cv2.imshow("Calibration Success - BEV View", warped)
        print("Matrix saved to bev_matrix.npy. You can now close the window.")
        cv2.waitKey(0)
    else:
        print("No marker found! Opening debug window...")
        # Draw the rejected shapes (helps us see if it almost found it)
        cv2.aruco.drawDetectedMarkers(frame, rejected, borderColor=(0, 0, 255))
        cv2.imshow("Debug Camera View", frame)
        print("Press any key on the image window to close.")
        cv2.waitKey(0)

    zed.close()


if __name__ == "__main__":
    run_calibration(marker_size_meters=0.19)  # Actual Paper Size

