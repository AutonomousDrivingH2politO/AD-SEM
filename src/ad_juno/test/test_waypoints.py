import math

import numpy as np
import pytest
import rclpy
from sensor_msgs.msg import NavSatFix

from ad_juno.waypoints_elaborate import WaypointRecorder

# ----------------------------------------------------------------
# ROS 2 setup
# ----------------------------------------------------------------


@pytest.fixture(scope="module", autouse=True)
def ros_init():
    rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


@pytest.fixture
def wp_node():
    # each test gets a clean node so state never bleeds between them
    recorder_node = WaypointRecorder()
    yield recorder_node
    recorder_node.destroy_node()


# ----------------------------------------------------------------
# Helpers
# ----------------------------------------------------------------

# ~5 m northward — small enough to slip under max_jump_m (8 m)
# but large enough to clear min_dist (1 m)
_5M_LAT = np.float64(0.000045)


def create_valid_fix_msg(lat, lon, sec=0):
    msg = NavSatFix()
    msg.status.status = 2
    msg.latitude = float(lat)
    msg.longitude = float(lon)
    msg.altitude = 10.0
    msg.header.stamp.sec = sec
    msg.header.stamp.nanosec = 0

    # 3x3 ENU covariance stored row-major; we only care about [0], [4], [8]
    cov = np.zeros(9, dtype=np.float64)
    cov[np.array([0, 4, 8])] = 0.001
    msg.position_covariance = cov.tolist()
    return msg


# ----------------------------------------------------------------
# Math utilities
# ----------------------------------------------------------------


def test_haversine_distance(wp_node):
    dist = wp_node.haversine_distance(0.0, 0.0, 1.0, 0.0)
    assert math.isclose(dist, 111195.0, rel_tol=0.01)
    assert wp_node.haversine_distance(45.0, 9.0, 45.0, 9.0) == 0.0


def test_bearing_deg(wp_node):
    assert math.isclose(wp_node.bearing_deg(0.0, 0.0, 1.0, 0.0), 0.0, abs_tol=0.1)
    assert math.isclose(wp_node.bearing_deg(0.0, 0.0, 0.0, 1.0), 90.0, abs_tol=0.1)


def test_angular_diff_deg(wp_node):
    # wrap-around: 350° and 10° are only 20° apart, not 340°
    assert wp_node.angular_diff_deg(350.0, 10.0) == 20.0
    assert wp_node.angular_diff_deg(10.0, 350.0) == 20.0
    assert wp_node.angular_diff_deg(90.0, 180.0) == 90.0


def test_extract_accuracy(wp_node):
    msg = NavSatFix()
    # east variance > north variance, so the function should pick east for h_acc
    cov = np.zeros(9, dtype=np.float64)
    cov[0] = 0.04  # east  → sqrt(0.04) = 0.20
    cov[4] = 0.01  # north → smaller, ignored
    cov[8] = 0.09  # up    → sqrt(0.09) = 0.30
    msg.position_covariance = cov.tolist()

    h_acc, v_acc = wp_node.extract_accuracy(msg)
    assert math.isclose(h_acc, 0.2, rel_tol=1e-5)
    assert math.isclose(v_acc, 0.3, rel_tol=1e-5)


# ----------------------------------------------------------------
# Quality gates
# ----------------------------------------------------------------


def test_gps_callback_rejects_no_fix_status(wp_node):
    msg = create_valid_fix_msg(45.0, 9.0)
    msg.status.status = -1
    wp_node.gps_callback(msg)
    assert len(wp_node.waypoints) == 0


def test_gps_callback_rejects_low_accuracy(wp_node):
    msg = create_valid_fix_msg(45.0, 9.0)
    bad_cov = np.zeros(9, dtype=np.float64)
    bad_cov[0] = 10.0  # h_acc = sqrt(10) ≈ 3.16 m, way over the 0.10 m limit
    msg.position_covariance = bad_cov.tolist()
    wp_node.gps_callback(msg)
    assert len(wp_node.waypoints) == 0


# ----------------------------------------------------------------
# Recording logic
# ----------------------------------------------------------------


def test_gps_callback_accepts_start_point(wp_node):
    wp_node.gps_callback(create_valid_fix_msg(45.0, 9.0, sec=1))
    assert len(wp_node.waypoints) == 1
    assert wp_node.records[0]["reason"] == "start"


def test_gps_callback_rejects_too_close(wp_node):
    wp_node.gps_callback(create_valid_fix_msg(45.0, 9.0, sec=1))
    wp_node.gps_callback(create_valid_fix_msg(45.0000001, 9.0, sec=2))  # ≈ 0.01 m
    assert len(wp_node.waypoints) == 1


def test_gps_callback_valid_sequence(wp_node):
    # _5M_LAT keeps us inside max_jump_m (8 m); dt=11 s gives speed ≈ 0.45 m/s
    wp_node.gps_callback(create_valid_fix_msg(45.0, 9.0, sec=1))
    assert len(wp_node.waypoints) == 1
    assert wp_node.records[0]["reason"] == "start"

    wp_node.gps_callback(create_valid_fix_msg(45.0000001, 9.0, sec=2))
    assert len(wp_node.waypoints) == 1

    wp_node.gps_callback(create_valid_fix_msg(45.0 + _5M_LAT, 9.0, sec=12))
    assert len(wp_node.waypoints) == 2
    assert wp_node.records[1]["reason"] == "distance"


def test_gps_callback_rejects_speed_spike(wp_node):
    # _5M_LAT keeps dist under max_jump_m so only the speed filter fires here
    wp_node.gps_callback(create_valid_fix_msg(45.0, 9.0, sec=1))
    wp_node.gps_callback(
        create_valid_fix_msg(45.0 + _5M_LAT, 9.0, sec=2)
    )  # 5 m/s > 3.0 m/s cap
    assert len(wp_node.waypoints) == 1


def test_gps_callback_rejects_outlier_jump(wp_node):
    wp_node.gps_callback(create_valid_fix_msg(45.0, 9.0, sec=1))
    wp_node.gps_callback(
        create_valid_fix_msg(45.0001, 9.0, sec=60)
    )  # ≈ 11 m, over max_jump_m
    assert len(wp_node.waypoints) == 1


# ----------------------------------------------------------------
# File I/O
# ----------------------------------------------------------------


def test_save_waypoints(wp_node, tmp_path):
    wp_node.output_dir = tmp_path / "tracks"

    wp_node.waypoints = np.array([[45.0, 9.0, 10.0]], dtype=np.float64).tolist()
    wp_node.records = [
        {
            "index": 0,
            "stamp_sec": np.float64(1.0),
            "status": 2,
            "lat": np.float64(45.0),
            "lon": np.float64(9.0),
            "alt": np.float64(10.0),
            "h_acc": np.float64(0.05),
            "v_acc": np.float64(0.05),
            "dist_from_prev_m": np.float64(0.0),
            "heading_change_deg": np.float64("nan"),
            "reason": "start",
        }
    ]

    wp_node.save_waypoints()

    assert (wp_node.output_dir / "track_waypoints.npy").exists()
    assert (wp_node.output_dir / "track_waypoints.csv").exists()

    saved = np.load(wp_node.output_dir / "track_waypoints.npy")
    expected = np.array([[45.0, 9.0, 10.0]], dtype=np.float64)
    assert saved.shape == (1, 3)
    np.testing.assert_allclose(saved, expected)


def test_save_waypoints_empty(wp_node, tmp_path):
    wp_node.output_dir = tmp_path / "tracks"
    wp_node.save_waypoints()
    assert not (wp_node.output_dir / "track_waypoints.npy").exists()
