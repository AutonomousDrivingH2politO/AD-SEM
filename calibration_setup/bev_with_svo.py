import os
import cv2 as cv
import numpy as np
import argparse

# Safely check if ZED SDK is installed
try:
    import pyzed.sl as sl
    ZED_AVAILABLE = True
except ImportError:
    ZED_AVAILABLE = False

# --- GLOBAL VARIABLES ---
clicked_points = []
original_image = None
display_image = None
depth_map = None 
calculated_M = None
waiting_for_save = False

# --- 1. YOUR ORIGINAL MATH FUNCTIONS ---

def get_vanishing_point(p1, p2, p3, p4):
    l1 = np.cross([p1[0], p1[1], 1], [p2[0], p2[1], 1])
    l2 = np.cross([p3[0], p3[1], 1], [p4[0], p4[1], 1])
    v = np.cross(l1, l2)
    if v[2] == 0: return None
    return (int(v[0] / v[2]), int(v[1] / v[2]))

def get_x_from_y(p1, p2, y):
    x1, y1 = p1
    x2, y2 = p2
    if y1 == y2: return x1
    return int(x1 + (y - y1) * (x2 - x1) / (y2 - y1))

def apply_birdseye_view(vp, left_line, right_line, img):
    h, w = img.shape[:2]
    top_y = vp[1] + 50
    bottom_y = h

    left_top_x = get_x_from_y(left_line[0], left_line[1], top_y)
    left_bottom_x = get_x_from_y(left_line[0], left_line[1], bottom_y)
    right_top_x = get_x_from_y(right_line[0], right_line[1], top_y)
    right_bottom_x = get_x_from_y(right_line[0], right_line[1], bottom_y)

    if left_bottom_x > right_bottom_x:
        left_top_x, right_top_x = right_top_x, left_top_x
        left_bottom_x, right_bottom_x = right_bottom_x, left_bottom_x

    src = np.float32([
        [left_top_x, top_y], [right_top_x, top_y],
        [right_bottom_x, bottom_y], [left_bottom_x, bottom_y]
    ])
    
    dst_margin = 300
    dst = np.float32([
        [dst_margin, 0], [w - dst_margin, 0],
        [w - dst_margin, h], [dst_margin, h]
    ])

    print("\n--- Calculated Transformation Points ---")
    print(f"src matrix = \n{src}")
    print(f"dst matrix = \n{dst}")

    M = cv.getPerspectiveTransform(src, dst)
    bev_image = cv.warpPerspective(img, M, (w, h))

    return bev_image, M, src

# --- 2. DEPTH LOOKUP (ONLY FOR DOTS) ---

def find_pixel_for_distance(depth_data, target_meters, w, h):
    if depth_data is None: return None
    center_x = w // 2
    for y in range(h - 1, 0, -1):
        err, depth_val = depth_data.get_value(center_x, y)
        if err == sl.ERROR_CODE.SUCCESS and np.isfinite(depth_val):
            if depth_val >= target_meters:
                return (center_x, y)
    return None

# --- 3. MOUSE CALLBACK ---

def mouse_callback(event, x, y, flags, param):
    global clicked_points, display_image, original_image, depth_map
    global calculated_M, waiting_for_save

    if waiting_for_save or event != cv.EVENT_LBUTTONDOWN:
        return

    if len(clicked_points) < 4:
        clicked_points.append((x, y))
        cv.circle(display_image, (x, y), 5, (0, 255, 0), -1)

        if len(clicked_points) == 2:
            cv.line(display_image, clicked_points[0], clicked_points[1], (255, 0, 0), 2)
        elif len(clicked_points) == 4:
            cv.line(display_image, clicked_points[2], clicked_points[3], (255, 0, 0), 2)
            vp = get_vanishing_point(clicked_points[0], clicked_points[1], clicked_points[2], clicked_points[3])

            if vp:
                cv.circle(display_image, vp, 8, (0, 0, 255), -1)
                
                # A. Apply the exact original transformation
                bev_result, M, src_pts = apply_birdseye_view(vp, (clicked_points[0], clicked_points[1]), 
                                                            (clicked_points[2], clicked_points[3]), original_image)

                # Draw the green trapezoid on original screen
                pts = src_pts.reshape((-1, 1, 2)).astype(np.int32)
                cv.polylines(display_image, [pts], isClosed=True, color=(0, 255, 0), thickness=2)

                # B. Use ZED depth to find points in ORIGINAL screen
                h, w = original_image.shape[:2]
                px_75 = find_pixel_for_distance(depth_map, 7.5, w, h)
                px_10 = find_pixel_for_distance(depth_map, 10.0, w, h)

                # C. Project, Draw DOTS ONLY, and Print Coordinates
                for px, dist, color in [(px_75, 7.5, (0, 255, 255)), (px_10, 10.0, (0, 165, 255))]:
                    if px:
                        # Draw DOT on original screen
                        cv.circle(display_image, px, 8, color, -1)
                        cv.putText(display_image, f"{dist}m", (px[0] + 15, px[1]), cv.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
                        
                        # Project that specific point to the BEV image
                        pt_original = np.array([[[px[0], px[1]]]], dtype=np.float32)
                        pt_bev = cv.perspectiveTransform(pt_original, M)
                        x_bev = int(pt_bev[0][0][0])
                        y_bev = int(pt_bev[0][0][1])
                        
                        # Print the coordinates to console
                        print(f"\n--- Depth Point for {dist}m ---")
                        print(f"Original Image Pixel: (x: {px[0]}, y: {px[1]})")
                        print(f"BEV Image Pixel:      (x: {x_bev}, y: {y_bev})")

                        # Draw DOT on BEV screen
                        if 0 <= y_bev < h and 0 <= x_bev < w:
                            cv.circle(bev_result, (x_bev, y_bev), 8, color, -1)
                            cv.putText(bev_result, f"{dist}m", (x_bev + 15, y_bev), cv.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)

                cv.imshow("Original Setup", display_image)
                cv.namedWindow("Bird's Eye View", cv.WINDOW_NORMAL)
                cv.imshow("Bird's Eye View", bev_result)
                
                calculated_M = M
                waiting_for_save = True
                print("\n>>> PRESS 'y' TO SAVE, 'n' TO EXIT <<<")
            else:
                clicked_points.clear()
        
        cv.imshow("Original Setup", display_image)

# --- 4. MAIN FLOW ---

def main():
    global original_image, display_image, depth_map, calculated_M, waiting_for_save
    
    parser = argparse.ArgumentParser()
    parser.add_argument("-zed", type=str, help="Path to SVO file", default=None)
    args = parser.parse_args()

    zed = None
    use_zed = False

    # SVO/Live Logic
    if args.zed and ZED_AVAILABLE:
        zed = sl.Camera()
        input_type = sl.InputType()
        input_type.set_from_svo_file(args.zed)
        init_params = sl.InitParameters(input_t=input_type)
        init_params.depth_mode = sl.DEPTH_MODE.ULTRA
        init_params.coordinate_units = sl.UNIT.METER
        
        if zed.open(init_params) == sl.ERROR_CODE.SUCCESS:
            tracking_params = sl.PositionalTrackingParameters()
            zed.enable_positional_tracking(tracking_params)
            plane = sl.Plane()
            img_sl, depth_sl = sl.Mat(), sl.Mat()
            
            print("Detecting floor in SVO...")
            while True:
                if zed.grab() == sl.ERROR_CODE.SUCCESS:
                    if zed.find_floor_plane(plane, sl.Transform()) == sl.ERROR_CODE.SUCCESS:
                        zed.retrieve_image(img_sl, sl.VIEW.LEFT)
                        zed.retrieve_measure(depth_sl, sl.MEASURE.DEPTH)
                        original_image = cv.cvtColor(img_sl.get_data(), cv.COLOR_BGRA2BGR)
                        depth_map = depth_sl
                        use_zed = True
                        break
                else: break
        else: print("Failed to open SVO.")

    # Static Fallback (Mac OS safe)
    if not use_zed:
        path = os.path.expanduser("/home/bylogix/AD-SEM/calibration_setup/left_20250510_160320.png")
        original_image = cv.imread(path)
        if original_image is None:
            print("Could not load image.")
            return

    display_image = original_image.copy()
    cv.namedWindow("Original Setup", cv.WINDOW_NORMAL)
    cv.setMouseCallback("Original Setup", mouse_callback)
    cv.imshow("Original Setup", display_image)

    while True:
        key = cv.waitKey(1) & 0xFF
        if waiting_for_save:
            if key == ord('y'):
                np.save("/home/bylogix/AD-SEM/calibration_setup/bev_matrix.npy", calculated_M)
                break
            elif key == ord('n') or key == 27: break
        elif key == 27: break

    cv.destroyAllWindows()
    if zed: zed.close()

if __name__ == "__main__":
    main()
