import cv2
import matplotlib.pyplot as plt
import numpy as np
import time
import os
from sklearn.linear_model import RANSACRegressor, LinearRegression
from sklearn.preprocessing import PolynomialFeatures
from sklearn.pipeline import make_pipeline

# v8 - oltre alle modifiche di v5, (situazioni critiche), il cambio della longitudinal distance avviene
#       per il prossimo frame se la longitudinal distance è maggiore di una threshold, ciò significa che
#       ci stiamo avvicinando ad una curva
#       E' stato aggiunto una funzione in grado di rimuovere i punti spuri dei bordi
#       Fixati alcuni errori su v4 e resa migliore la rimozione dei bordi spuri
#       Eliminati i fake edges.
#       Aggiunta una funzione in grado di calcolare la curvatura della traiettoria (da migliorare con anomaly detection)
#       Modifica della longitudinal distance quando ci approcciamo ad una curva

SIMULATION = False
LANE_METERS = 10

# WARNING: These pixel rows (515 and 580) MUST be recalculated physically
Y_METERS = {10.0: 222,
            7.5: 411,
            }

LANE_PIXELS = None
LATERAL_DISTANCE = 0
scale_factor = None
black_regions = None
y_black = None
prev_curvature = None

#  load bev and y meters data
try:
    bev_data = np.load("/home/bylogix/AD-SEM/calibration_setup/bev_matrix.npz")
    BEV_MATRIX = bev_data['matrix']
    Y_METERS[7.5] = int(bev_data['y_7_5'])
    Y_METERS[10.0] = int(bev_data['y_10'])
    print("SUCCESS: Custom bev_matrix.npz loaded correctly with 7.5m and 10m.")
except FileNotFoundError:
    try:
        # in case the usage of old bev calibration code without y meters
        BEV_MATRIX = np.load(
            "/home/bylogix/AD-SEM/calibration_setup/bev_matrix.npy")
        print("bev_matrix.npy loaded")
    except FileNotFoundError:
        print("WARNING: bev_matrix.npy not found. Using fallback hardcoded matrix.")
        BEV_MATRIX = None


def load_camera_calib(sim=True):
    if not sim:
        # ZED2i
        # [LEFT_CAM_HD] 1280x720
        fx = 532.655
        fy = 532.595
        cx = 621.155
        cy = 349.6815
        k1 = -0.0435025
        k2 = 0.0158395
        p1 = 0.0012899
        p2 = -0.00129192
        k3 = -0.00655099
        mtx = [[fx,     0, cx],
               [0,      fy,  cy],
               [0,          0,      1]]
        dist = [k1,
                k2,
                p1,
                p2,
                k3]

    else:
        mtx = [[1395.35, 0, 640],
               [0, 1395.35, 360],
               [0, 0, 1]]
        dist = [0, 0, 0, 0, 0]
    return np.array(mtx), np.array(dist)


def undistort(img, mtx, dist):
    undistort_img = cv2.undistort(img, mtx, dist, None, mtx)
    return undistort_img


def warp_image(img, warp_shape, src, dst):
    M = cv2.getPerspectiveTransform(src, dst)
    invM = cv2.getPerspectiveTransform(dst, src)

    warped = cv2.warpPerspective(img, M, warp_shape, flags=cv2.INTER_CUBIC)
    return warped, M, invM


def eye_bird_view(img, mtx, dist, d=450):
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
            (50.0, 675.0),
            (1230.0, 675.0)
        ])
        dst = np.float32([
            (xsize - d, 0),
            (d, 0),
            (d, ysize),
            (xsize - d, ysize)
        ])
        warped, _, _ = warp_image(undist, (xsize, ysize), src, dst)

    return warped


def processing_mask(mask, img, show=False, d=450):
    global black_regions, y_black
    mtx, dist = load_camera_calib(sim=SIMULATION)
    warped = eye_bird_view(mask, mtx, dist, d=d)

    if black_regions is None:
        img_warped = eye_bird_view(img, mtx, dist, d=d)
        black_regions = cv2.inRange(
            img_warped, np.array([0, 0, 0]), np.array([0, 0, 0]))
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (2, 2))
        black_regions = cv2.dilate(black_regions, kernel, iterations=1)
        nonzero_rows = np.nonzero(black_regions[:, 0] == 255)[0]
        y_black = int(np.min(nonzero_rows)) if len(nonzero_rows) > 0 else 0

    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (14, 14))
    res_morph = cv2.morphologyEx(warped, cv2.MORPH_CLOSE, kernel)

    _, res_morph_th = cv2.threshold(res_morph, 0, 255, cv2.THRESH_BINARY)
    line_edges = cv2.Canny(res_morph_th, 100, 100)

    vertical_edges = np.zeros_like(line_edges)
    vertical_edges[:, [0, -1]] = 255

    combined_edges = cv2.bitwise_and(warped, vertical_edges)
    _, combined_edges = cv2.threshold(
        combined_edges, 0, 255, cv2.THRESH_BINARY)
    line_edges -= black_regions
    line_edges = cv2.bitwise_or(line_edges, combined_edges)

    if y_black is not None and y_black > 10:
        if any(combined_edges[[y_black, y_black-10], 0] == 255):
            line_edges[:, 0] = 255
        elif any(combined_edges[[y_black, y_black-10], -1] == 255):
            line_edges[:, -1] = 255

    _, line_edges = cv2.threshold(line_edges, 2, 255, cv2.THRESH_BINARY)
    if show:
        plt.imshow(line_edges)
        plt.show()
    return line_edges


def merge_close_edges(lst, tol=10):
    result = []
    temp = []
    for x in lst:
        if temp and abs(x - temp[0]) > tol:
            if len(temp) == 1:
                result.append(temp[0])
            else:
                result.append(sum(temp) // len(temp))
            temp = []
        temp.append(x)

    if temp:
        result.append(sum(temp) // len(temp))
    return result


def computing_mid_point(line_edges, y):
    white_pixels = np.nonzero(line_edges[y, :])[0]
    white_pixels = merge_close_edges(white_pixels)
    if len(white_pixels) == 0:
        return None
    elif len(white_pixels) == 1:
        if LANE_PIXELS is not None:
            white_pixels = np.nonzero(line_edges[y, :])[0]
            if white_pixels[0] > line_edges.shape[1]//2:
                x_coords_points = white_pixels[0]-LANE_PIXELS, white_pixels[0]
            else:
                x_coords_points = white_pixels[0], white_pixels[0]+LANE_PIXELS
        else:
            return None
    elif len(white_pixels) == 2 and (0 in white_pixels or line_edges.shape[1]-1 in white_pixels):
        if LANE_PIXELS is None:
            return
        if 0 in white_pixels and white_pixels[-1] >= line_edges.shape[1]//2:
            x_coords_points = white_pixels[-1]-LANE_PIXELS, white_pixels[-1]
        elif 0 in white_pixels:
            return -np.inf
        if line_edges.shape[1]-1 in white_pixels and white_pixels[0] <= line_edges.shape[1]//2:
            x_coords_points = white_pixels[0], white_pixels[0]+LANE_PIXELS
        elif line_edges.shape[1]-1 in white_pixels:
            return +np.inf

    elif len(white_pixels) >= 2:
        max_diff = float('-inf')
        max_diff_indices = None

        for i in range(len(white_pixels) - 1):
            diff = abs(white_pixels[i] - white_pixels[i+1])
            if diff > max_diff:
                max_diff = diff
                max_diff_indices = (i, i+1)
        x_coords_points = white_pixels[max_diff_indices[0]
                                       ], white_pixels[max_diff_indices[1]]
    else:
        x_coords_points = white_pixels[0], white_pixels[1]
    return x_coords_points


def computing_mid_pointS(line_edges, y, th_y=300, n_point=6):
    y_values = [int(x) for x in np.linspace(th_y, y, n_point)[:-1]]
    midpoints = []
    for y_act in y_values:
        x_coords_points = computing_mid_point(line_edges, y_act)
        if x_coords_points is not None and x_coords_points != +np.inf and x_coords_points != -np.inf:
            posm = y_act, (x_coords_points[1] + x_coords_points[0])//2
            midpoints.append(posm)
    return midpoints


class TemporalSmoother:
    def __init__(self, alpha=0.4):
        # alpha=1.0 is no smoothing, alpha=0.1 is heavy smoothing
        self.alpha = alpha
        self.prev_coeffs = None

    def smooth(self, coeffs):
        if coeffs is not None:
            if self.prev_coeffs is None:
                self.prev_coeffs = coeffs
            else:
                self.prev_coeffs = (self.alpha * coeffs) + ((1 - self.alpha) * self.prev_coeffs)
        return self.prev_coeffs

temporal_smoother = TemporalSmoother(alpha=0.4)


def smooth_midpoints_spline(midpoints, degree=1, residual_threshold=10.0):
    global temporal_smoother
    
    if not midpoints or len(midpoints) < degree + 1:
        if temporal_smoother.prev_coeffs is not None and midpoints:
            poly = np.poly1d(temporal_smoother.prev_coeffs)
            ys = np.array(midpoints)[:, 0]
            smoothed_xs = poly(ys)
            return [(int(y), int(round(x))) for y, x in zip(ys, smoothed_xs)]
        return midpoints

    arr = np.array(midpoints, dtype=float)
    ys = arr[:, 0]
    xs = arr[:, 1]

    model = make_pipeline(
        PolynomialFeatures(degree=degree, include_bias=True),
        RANSACRegressor(
            estimator=LinearRegression(),
            residual_threshold=residual_threshold,
            min_samples=degree + 1,
            max_trials=200,
            random_state=42,
        )
    )
    
    try:
        model.fit(ys.reshape(-1, 1), xs)
        
        # 1. Extract raw coefficients
        ransac = model.named_steps["ransacregressor"]
        lin = ransac.estimator_
        poly_feat = model.named_steps["polynomialfeatures"]
        
        coef = lin.coef_.copy()
        coef[0] += lin.intercept_
        powers = poly_feat.powers_.flatten()
        coeffs = coef[np.argsort(powers)[::-1]]

        # 2. Apply Temporal Smoothing
        smoothed_coeffs = temporal_smoother.smooth(coeffs)
        poly = np.poly1d(smoothed_coeffs)
        
        # 3. Project the smoothed curve back onto the original Y coordinates
        smoothed_xs = poly(ys)
        return [(int(y), int(round(x))) for y, x in zip(ys, smoothed_xs)]
        
    except ValueError:
        # If RANSAC fails entirely on a chaotic frame, fall back to historical memory
        if temporal_smoother.prev_coeffs is not None:
            poly = np.poly1d(temporal_smoother.prev_coeffs)
            smoothed_xs = poly(ys)
            return [(int(y), int(round(x))) for y, x in zip(ys, smoothed_xs)]
        return midpoints

def computing_delta(midpoints, th_straight=20):
    global prev_curvature
    midpoints = np.array(midpoints)
    next_point = midpoints[-1]

    x = midpoints[:, 1]
    x_mean = np.mean(x)
    x_stdev = np.sqrt((np.var(x)))
    midpoints = np.stack(
        [p for p in midpoints if abs(p[1] - x_mean) < 1.5*x_stdev])
    if next_point[1] not in midpoints[:, 1]:
        midpoints = np.vstack((midpoints, next_point))

        # midpoints = np.array(normal_points)

    delta_x = next_point[1] - midpoints[:, 1]

    mean_delta_x = np.mean(delta_x)
    print("\t ---------- \t")
    print('Sum of delta_x =', -mean_delta_x)

    if prev_curvature is not None:
        if (prev_curvature == 'left' and mean_delta_x < 0) or (prev_curvature == 'right' and mean_delta_x > 0):
            return prev_curvature, midpoints

    if abs(mean_delta_x) < th_straight:
        curvature = 'straight'
    elif mean_delta_x > 0:
        curvature = 'left'
    elif mean_delta_x < 0:
        curvature = 'right'

    prev_curvature = curvature

    print(f"{curvature = }")
    print(f"{midpoints = }")
    return curvature, midpoints

def computing_lateral_distance(line_edges, show=False):
    global LANE_PIXELS
    global LATERAL_DISTANCE
    global scale_factor
    
    if prev_curvature is not None:
        if prev_curvature != "straight":
            y = Y_METERS[7.5]
            long_dist = 7.5
        else:
            y = Y_METERS[7.5]
            long_dist = 7.5
    else:
        y = Y_METERS[7.5]
        long_dist = 7.5
        
    x_coords_points = computing_mid_point(line_edges, y)

    if x_coords_points is None:
        return LATERAL_DISTANCE, long_dist, None
    if x_coords_points == -np.inf or x_coords_points == np.inf:
        return x_coords_points, long_dist, None

    # 1. Gather all raw midpoints
    posm = y, (x_coords_points[1] + x_coords_points[0])//2
    midpoints = computing_mid_pointS(line_edges, y)
    midpoints.append(posm)

    # 2. Smooth the trajectory (using degree=2 for curves)
    smoothed_midpoints = smooth_midpoints_spline(midpoints, degree=2)

    # 3. Select the target waypoint from the smoothed curve.
    # index -1 corresponds to y=411 (posm). Change to 0 if you want y=300.
    target_smoothed_point = smoothed_midpoints[-1]
    smoothed_y, smoothed_x = target_smoothed_point

    # 4. Calculate lateral distance using the SMOOTHED x
    middle_image = line_edges.shape[1]//2
    lateral_distance = smoothed_x - middle_image
    
    # 5. Keep your existing scaling logic intact
    if not LANE_PIXELS:
        LANE_PIXELS = x_coords_points[1] - x_coords_points[0]
        scale_factor = LANE_METERS / LANE_PIXELS

    later_distance_meters = lateral_distance * scale_factor
    LATERAL_DISTANCE = later_distance_meters

    if show:
        for p in smoothed_midpoints:
            cv2.circle(line_edges, p[::-1], 2, (255, 255, 255), 2)
        plt.imshow(line_edges)
        plt.show()

    return later_distance_meters, long_dist, smoothed_midpoints