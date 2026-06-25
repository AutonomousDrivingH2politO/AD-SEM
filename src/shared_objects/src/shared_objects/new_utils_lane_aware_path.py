import cv2
import matplotlib.pyplot as plt
import numpy as np
import time
import os

# Lane-aware BEV edge extraction.
#
# Works with the seg_node_plus2 dual-channel mask encoding:
#   0   = non-drivable background
#   128 = drivable road
#   255 = lane line
#
# Edge strategy based on how many lane lines are visible in BEV:
#   0 lanes → Canny edges of the road blob boundary
#   1 lane  → that lane line + Canny road edge on the opposite side
#   2 lanes → both lane lines used directly as edges

SIMULATION = False
LANE_METERS = 10

Y_METERS = {10.0: 222,
            7.5: 411,
            }
LANE_PIXELS = None
LATERAL_DISTANCE = 0
scale_factor = None
black_regions = None
y_black = None
prev_curvature = None
D_param = 450

try:
    bev_data = np.load("/home/bylogix/AD-SEM/calibration_setup/bev_matrix.npz")
    BEV_MATRIX = bev_data['matrix']
    Y_METERS[7.5] = int(bev_data['y_7_5'])
    Y_METERS[10.0] = int(bev_data['y_10'])
    print("SUCCESS: Custom bev_matrix.npz loaded correctly with 7.5m and 10m.")
except FileNotFoundError:
    try:
        BEV_MATRIX = np.load(
            "/home/bylogix/AD-SEM/calibration_setup/bev_matrix.npy")
        print("bev_matrix.npy loaded")
    except FileNotFoundError:
        print("WARNING: bev_matrix.npy not found. Using fallback hardcoded matrix.")
        BEV_MATRIX = None


def load_camera_calib(sim=True):
    if not sim:
        # ZED2i [LEFT_CAM_HD] 1280x720
        fx = 532.655
        fy = 532.595
        cx = 621.155
        cy = 349.6815
        k1 = -0.0435025
        k2 = 0.0158395
        p1 = 0.0012899
        p2 = -0.00129192
        k3 = -0.00655099
        mtx = [[fx, 0,  cx],
               [0,  fy, cy],
               [0,  0,  1]]
        dist = [k1, k2, p1, p2, k3]
    else:
        mtx  = [[1395.35, 0, 640],
                [0, 1395.35, 360],
                [0, 0,       1]]
        dist = [0, 0, 0, 0, 0]
    return np.array(mtx), np.array(dist)


def undistort(img, mtx, dist):
    return cv2.undistort(img, mtx, dist, None, mtx)


def warp_image(img, warp_shape, src, dst):
    M    = cv2.getPerspectiveTransform(src, dst)
    invM = cv2.getPerspectiveTransform(dst, src)
    warped = cv2.warpPerspective(img, M, warp_shape, flags=cv2.INTER_CUBIC)
    return warped, M, invM


def eye_bird_view(img, mtx, dist, d=D_param):
    ysize = img.shape[0]
    xsize = img.shape[1]
    undist = undistort(img, mtx, dist)

    if BEV_MATRIX is not None:
        warped = cv2.warpPerspective(
            undist, BEV_MATRIX, (xsize, ysize), flags=cv2.INTER_CUBIC)
    else:
        src = np.float32([
            (700.0, 395.0),
            (605.0, 395.0),
            (50.0,  675.0),
            (1230.0, 675.0)
        ])
        dst = np.float32([
            (xsize - d, 0),
            (d,         0),
            (d,         ysize),
            (xsize - d, ysize)
        ])
        warped, _, _ = warp_image(img, (xsize, ysize), src, dst)

    return warped


def merge_close_edges(lst, tol=10):
    result = []
    temp   = []
    for x in lst:
        if temp and abs(x - temp[0]) > tol:
            result.append(sum(temp) // len(temp))
            temp = []
        temp.append(x)
    if temp:
        result.append(sum(temp) // len(temp))
    return result


def _count_lanes_in_bev(lane_bev_bin):
    """
    Count distinct lane lines by projecting the BEV lane mask onto the x-axis
    and clustering non-zero columns.

    Uses the bottom 3/4 of the BEV (nearest to the vehicle) where lane
    detection is most reliable.

    Returns (n_lanes, lane_cluster_x_positions).
    """
    h = lane_bev_bin.shape[0]
    scan = lane_bev_bin[h // 4:, :]
    col_profile = np.sum(scan > 0, axis=0)
    min_votes   = max(1, int(scan.shape[0] * 0.05))  # ≥5 % of rows must vote
    lane_cols   = np.where(col_profile >= min_votes)[0]
    clusters    = merge_close_edges(lane_cols.tolist(), tol=40)
    return len(clusters), clusters


def _road_canny_edges(road_bev, black_regions):
    """Morphological close + Canny on the road blob, minus BEV void artifacts."""
    k = cv2.getStructuringElement(cv2.MORPH_RECT, (14, 14))
    closed = cv2.morphologyEx(road_bev, cv2.MORPH_CLOSE, k)
    _, binary = cv2.threshold(closed, 0, 255, cv2.THRESH_BINARY)
    edges = cv2.Canny(binary, 100, 300)
    if black_regions is not None:
        edges = cv2.subtract(edges, black_regions)
    return edges


def _thicken_lanes(lane_bev_bin):
    """Dilate thin lane lines so they register clearly when scanning rows."""
    k = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    return cv2.dilate(lane_bev_bin, k, iterations=1)


def _road_edges_valid(road_edges):
    """
    Return True when road Canny edges produce at least two distinct boundary
    clusters at the reference row — i.e. a usable left + right pair exists.
    A single cluster means one wall is missing; zero means the road blob is gone.
    """
    ref_y = min(Y_METERS[7.5], road_edges.shape[0] - 1)
    raw   = np.nonzero(road_edges[ref_y, :])[0]
    return len(merge_close_edges(raw.tolist(), tol=10)) >= 2


def processing_mask(mask, img, show=False, d=D_param):
    """
    Convert the encoded segmentation mask to a BEV edge image.

    mask encoding (seg_node_plus2):
        0   = non-drivable
        128 = drivable road
        255 = lane line

    Primary path (road edges valid):
        Identical to the original path_planning.py — Canny on the road blob.

    Fallback (road edges lost, i.e. < 2 boundary clusters at reference row):
        2 lanes  →  both lane lines as edges
        1 lane   →  that lane line + road Canny on the opposite side
        0 lanes  →  road Canny as best effort (same as primary, no change)
    """
    global black_regions, y_black
    mtx, dist = load_camera_calib(sim=SIMULATION)

    # Decode road (128 + 255 pixels) and lane (255 pixels only) channels
    road_raw = np.where(mask >= 64,  np.uint8(255), np.uint8(0))
    lane_raw = np.where(mask >= 200, np.uint8(255), np.uint8(0))

    road_bev = eye_bird_view(road_raw, mtx, dist, d=d)
    lane_bev = eye_bird_view(lane_raw, mtx, dist, d=d)

    # Re-binarise after cubic interpolation artefacts
    _, road_bev_bin = cv2.threshold(road_bev, 64,  255, cv2.THRESH_BINARY)
    _, lane_bev_bin = cv2.threshold(lane_bev, 127, 255, cv2.THRESH_BINARY)

    # BEV void mask — computed once from the first RGB frame
    if black_regions is None:
        img_warped = eye_bird_view(img, mtx, dist, d=d)
        black_regions = cv2.inRange(
            img_warped, np.array([0, 0, 0]), np.array([0, 0, 0]))
        k = cv2.getStructuringElement(cv2.MORPH_RECT, (2, 2))
        black_regions = cv2.dilate(black_regions, k, iterations=1)
        nonzero_rows = np.nonzero(black_regions[:, 0] == 255)[0]
        y_black = int(np.min(nonzero_rows)) if len(nonzero_rows) > 0 else 0

    # --- Primary: road boundary Canny (same as current path_planning.py) ---
    road_edges = _road_canny_edges(road_bev_bin, black_regions)

    if _road_edges_valid(road_edges):
        # Road boundary is clean → no change from current behaviour
        line_edges = road_edges
    else:
        # Road edges lost → try lane-line fallback
        n_lanes, lane_clusters = _count_lanes_in_bev(lane_bev_bin)
        mid_x = road_bev_bin.shape[1] // 2

        if n_lanes >= 2:
            # Both lane boundaries visible → use them directly
            line_edges = _thicken_lanes(lane_bev_bin)

        elif n_lanes == 1:
            # One lane visible → combine it with the road edge on the other side
            lane_x     = lane_clusters[0]
            lane_thick = _thicken_lanes(lane_bev_bin)
            line_edges = np.zeros_like(road_edges)
            if lane_x < mid_x:
                line_edges[:, :mid_x] = lane_thick[:, :mid_x]
                line_edges[:, mid_x:] = road_edges[:, mid_x:]
            else:
                line_edges[:, mid_x:] = lane_thick[:, mid_x:]
                line_edges[:, :mid_x] = road_edges[:, :mid_x]

        else:
            # No lanes either → road edges are the best we have
            line_edges = road_edges

    # Remove residual BEV void artefacts and binarise
    if black_regions is not None:
        line_edges = cv2.subtract(line_edges, black_regions)
    _, line_edges = cv2.threshold(line_edges, 2, 255, cv2.THRESH_BINARY)

    if show:
        plt.imshow(line_edges)
        plt.show()
    return line_edges


def computing_mid_point(line_edges, y):
    white_pixels = np.nonzero(line_edges[y, :])[0]
    white_pixels = merge_close_edges(white_pixels.tolist())
    if len(white_pixels) == 0:
        return None
    elif len(white_pixels) == 1:
        if LANE_PIXELS is not None:
            raw = np.nonzero(line_edges[y, :])[0]
            if raw[0] > line_edges.shape[1] // 2:
                x_coords_points = raw[0] - LANE_PIXELS, raw[0]
            else:
                x_coords_points = raw[0], raw[0] + LANE_PIXELS
        else:
            return None
    elif len(white_pixels) == 2 and (0 in white_pixels or line_edges.shape[1] - 1 in white_pixels):
        if LANE_PIXELS is None:
            return None
        if 0 in white_pixels and white_pixels[-1] >= line_edges.shape[1] // 2:
            x_coords_points = white_pixels[-1] - LANE_PIXELS, white_pixels[-1]
        elif 0 in white_pixels:
            return -np.inf
        if line_edges.shape[1] - 1 in white_pixels and white_pixels[0] <= line_edges.shape[1] // 2:
            x_coords_points = white_pixels[0], white_pixels[0] + LANE_PIXELS
        elif line_edges.shape[1] - 1 in white_pixels:
            return +np.inf
    elif len(white_pixels) >= 2:
        max_diff = float('-inf')
        max_diff_indices = None
        for i in range(len(white_pixels) - 1):
            diff = abs(white_pixels[i] - white_pixels[i + 1])
            if diff > max_diff:
                max_diff = diff
                max_diff_indices = (i, i + 1)
        x_coords_points = white_pixels[max_diff_indices[0]], white_pixels[max_diff_indices[1]]
    else:
        x_coords_points = white_pixels[0], white_pixels[1]
    return x_coords_points


def computing_mid_pointS(line_edges, y, th_y=300, n_point=6):
    y_values  = [int(x) for x in np.linspace(th_y, y, n_point)[:-1]]
    midpoints = []
    for y_act in y_values:
        x_coords_points = computing_mid_point(line_edges, y_act)
        if x_coords_points is not None and x_coords_points != +np.inf and x_coords_points != -np.inf:
            posm = y_act, (x_coords_points[1] + x_coords_points[0]) // 2
            midpoints.append(posm)
    return midpoints


def computing_delta(midpoints, th_straight=20):
    global prev_curvature
    midpoints  = np.array(midpoints)
    next_point = midpoints[-1]

    x       = midpoints[:, 1]
    x_mean  = np.mean(x)
    x_stdev = np.sqrt(np.var(x))
    midpoints = np.stack(
        [p for p in midpoints if abs(p[1] - x_mean) < 1.5 * x_stdev])
    if next_point[1] not in midpoints[:, 1]:
        midpoints = np.vstack((midpoints, next_point))

    delta_x      = next_point[1] - midpoints[:, 1]
    mean_delta_x = np.mean(delta_x)

    if prev_curvature is not None:
        if (prev_curvature == 'left'  and mean_delta_x < 0) or \
           (prev_curvature == 'right' and mean_delta_x > 0):
            return prev_curvature, midpoints

    if abs(mean_delta_x) < th_straight:
        curvature = 'straight'
    elif mean_delta_x > 0:
        curvature = 'left'
    else:
        curvature = 'right'

    prev_curvature = curvature
    return curvature, midpoints


# --- Noise / obstacle filtering (carried over from new_utils_midpoint_avoidance_path) ---

pre_midpoints = []


def is_lane_noisy(midpoints, noise_threshold=70):
    if not midpoints or len(midpoints) < 3:
        return True
    x_coords      = np.array([p[1] for p in midpoints])
    x_median      = np.median(x_coords)
    abs_deviations = np.abs(x_coords - x_median)
    mad           = np.median(abs_deviations)
    return mad > noise_threshold


def filter_midpoints_obstacle(midpoints, obstacle_seen):
    global pre_midpoints

    if not midpoints or len(midpoints) < 3:
        return midpoints

    noisy_condition = is_lane_noisy(midpoints, noise_threshold=70)

    if obstacle_seen:
        if not noisy_condition:
            x_coords       = np.array([p[1] for p in midpoints])
            x_median       = np.median(x_coords)
            abs_deviations = np.abs(x_coords - x_median)
            mad            = np.median(abs_deviations)
            rebel_margin   = max(mad * 3, 70)

            bottomest_point = max(midpoints, key=lambda p: p[0])
            if abs(bottomest_point[1] - x_median) > rebel_margin:
                non_rebels = [p for p in midpoints if abs(p[1] - x_median) <= rebel_margin]
                if len(non_rebels) > 0:
                    bottomest_non_rebel = max(non_rebels, key=lambda p: p[0])
                    new_point = (bottomest_point[0], bottomest_non_rebel[1])
                    for i, p in enumerate(midpoints):
                        if p == bottomest_point:
                            midpoints[i] = new_point
                            break

            pre_midpoints = midpoints.copy()
        else:
            if pre_midpoints:
                pre_x_mean   = np.mean([p[1] for p in pre_midpoints])
                sorted_by_diff = sorted(midpoints, key=lambda p: abs(p[1] - pre_x_mean))
                least_var_3  = sorted_by_diff[:3]
                bottomest_of_3   = max(least_var_3, key=lambda p: p[0])
                target_x         = bottomest_of_3[1]
                bottomest_current = max(midpoints, key=lambda p: p[0])
                target_y          = bottomest_current[0]
                new_point         = (target_y, target_x)
                for i, p in enumerate(midpoints):
                    if p == bottomest_current:
                        midpoints[i] = new_point
                        break

    return midpoints


def computing_lateral_distance(line_edges, obstacle_seen=False, show=False):
    global LANE_PIXELS
    global LATERAL_DISTANCE
    global scale_factor

    y        = Y_METERS[7.5]
    long_dist = 7.5

    x_coords_points = computing_mid_point(line_edges, y)

    if x_coords_points is None:
        return LATERAL_DISTANCE, long_dist, None
    if x_coords_points == -np.inf:
        return -np.inf, long_dist, None
    elif x_coords_points == np.inf:
        return np.inf, long_dist, None

    posm           = y, (x_coords_points[1] + x_coords_points[0]) // 2
    middle_image   = line_edges.shape[1] // 2
    lateral_distance = posm[1] - middle_image

    if not LANE_PIXELS:
        LANE_PIXELS  = x_coords_points[1] - x_coords_points[0]
        scale_factor = LANE_METERS / LANE_PIXELS

    later_distance_meters = lateral_distance * scale_factor
    LATERAL_DISTANCE      = later_distance_meters

    midpoints = computing_mid_pointS(line_edges, y)
    midpoints.append(posm)
    midpoints = filter_midpoints_obstacle(midpoints, obstacle_seen)

    if show:
        for p in midpoints:
            cv2.circle(line_edges, p[::-1], 2, (255, 255, 255), 2)
        plt.imshow(line_edges)
        plt.show()

    return later_distance_meters, long_dist, midpoints
