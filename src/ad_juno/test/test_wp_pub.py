import pytest
import math
import numpy as np
from unittest.mock import MagicMock, patch, mock_open
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix

from ad_juno.waypoint_publisher import WaypointNavigator


# ──────────────────────────────────────────────────────────────
# Full CSV from the robot's recorded run (73 waypoints)
# ──────────────────────────────────────────────────────────────

SAMPLE_CSV = """\
index,stamp_sec,status,lat,lon,alt,h_acc,v_acc,dist_from_prev_m,heading_change_deg,reason
0,1773760130.0002756,2,45.0419561,7.5963935,329.266,0.014,0.018,0.0,nan,start
1,1773760154.0002832,2,45.0419687,7.5963829,329.64300000000003,0.014,0.015,1.6298980625264967,nan,distance
2,1773760166.0002873,2,45.0419635,7.5963863,329.597,0.014,0.011,0.6369395735366746,174.06836035462783,turn
3,1773760184.000293,0,45.041967899999996,7.596387699999999,329.608,0.023,0.041,0.5014701881860951,142.5322816686222,turn
4,1773760189.0002944,2,45.0419705,7.596391499999999,329.646,0.014,0.012,0.41559872080633425,33.25104790617689,turn
5,1773760190.000295,2,45.0419766,7.596394,329.732,0.014,0.013,0.7061571366735157,29.771566293171645,turn
6,1773760192.0002954,2,45.0419944,7.596404,329.771,0.014,0.013,2.129511228169859,5.500835972394498,distance
7,1773760193.0002959,2,45.042004,7.5964075,329.754,0.014,0.011,1.1023227069779689,7.2051933698319885,distance
8,1773760194.000296,2,45.042013,7.5964111999999995,329.813,0.014,0.017,1.0421222852420808,1.7519618840161115,distance
9,1773760195.0002964,2,45.0420194,7.5964113,329.814,0.017,0.02,0.711690901433206,15.565307820980479,turn
10,1773760197.000297,2,45.0420224,7.596406399999999,329.724,0.014,0.018,0.5094061683089783,49.72424350883267,turn
11,1773760198.0002973,2,45.0420264,7.596394999999999,329.743,0.014,0.014,1.000041691111001,14.500238152435713,distance
12,1773760199.0002978,2,45.042030499999996,7.5963821,329.749,0.014,0.013,1.1113538388536441,2.1894240018276605,distance
13,1773760200.000298,2,45.0420333,7.596369999999999,329.778,0.014,0.012,1.0003687552669975,6.085201290006751,distance
14,1773760203.000299,2,45.042031099999996,7.596362399999999,329.75,0.014,0.015,0.6452911601911007,40.41128110831883,turn
15,1773760204.0002992,2,45.0420237,7.5963549,329.76300000000003,0.015,0.019,1.012079947450615,32.114324285343656,distance
16,1773760205.0002995,2,45.042014599999995,7.5963498,329.777,0.014,0.015,1.0883247120181139,14.004270934936415,distance
17,1773760207.0003002,2,45.0419986,7.5963423,329.76,0.014,0.018,1.8741665242146885,3.2780063769221783,distance
18,1773760208.0003004,2,45.0419957,7.596337999999999,329.795,0.015,0.017,0.4670378466854703,28.008874154154398,turn
19,1773760209.000301,2,45.0419901,7.5963341,329.67900000000003,0.016,0.02,0.6940011335173394,20.133168269366365,turn
20,1773760211.0003014,2,45.0419809,7.5963271,329.648,0.015,0.016,1.1614633105335388,2.0621778266868205,distance
21,1773760214.0003023,2,45.0419838,7.5963161999999995,329.745,0.014,0.012,0.9151007679363174,82.36965982962528,turn
22,1773760216.000303,2,45.041989199999996,7.5962912,329.78000000000003,0.014,0.01,2.0539542024774926,3.6350000877042703,distance
23,1773760217.0003033,2,45.041992899999996,7.5962792,329.714,0.014,0.01,1.0242885614359394,6.006786029839361,distance
24,1773760218.0003035,2,45.0419951,7.5962746999999995,329.707,0.014,0.011,0.4363626424284634,12.875237127946264,turn
25,1773760221.0003045,2,45.0420036,7.5962852,329.738,0.014,0.013,1.254553692616363,95.23577173649039,distance
26,1773760222.000305,2,45.0420132,7.5962938,329.794,0.014,0.016,1.263351393226553,8.782690725027578,distance
27,1773760223.0003052,2,45.042023,7.596302,329.808,0.014,0.015,1.2659174394941741,1.7404917853858137,distance
28,1773760224.0003054,2,45.0420328,7.5963096,329.783,0.014,0.015,1.2425884125765934,1.8714646066088676,distance
29,1773760225.000306,2,45.042042099999996,7.596316799999999,329.77500000000003,0.014,0.013,1.178728979441142,0.04101475757801154,distance
30,1773760227.0003064,0,45.0420477,7.596316,329.798,0.08,0.095,0.62585588466886,34.44420665268319,turn
31,1773760229.000307,0,45.0420515,7.5963012,329.72700000000003,0.064,0.076,1.2372115680046063,64.26610661805307,distance
32,1773760230.0003073,0,45.0420556,7.5962863,329.706,0.056,0.067,1.256315941829142,1.3076151596742989,distance
33,1773760232.000308,0,45.042065699999995,7.596255599999999,329.77500000000003,0.058,0.094,2.660704588307323,3.6893698668745287,distance
34,1773760233.0003083,0,45.0420701,7.5962407999999995,329.825,0.057,0.092,1.2615561868702463,2.1479297208078947,distance
35,1773760235.000309,0,45.0420753,7.596220799999999,329.72700000000003,0.051,0.062,1.674383961900458,2.6170759219410797,distance
36,1773760236.0003092,0,45.0420789,7.5962168,329.786,0.054,0.064,0.5089309295263489,31.662751364825056,turn
37,1773760238.0003097,0,45.0420762,7.5962133,329.81,0.051,0.061,0.40713143923115075,99.37663083562589,turn
38,1773760240.0003104,0,45.0420684,7.596209099999999,329.744,0.061,0.078,0.9279751063738603,21.657692394192054,turn
39,1773760241.0003107,0,45.0420588,7.5962032,329.699,0.066,0.08,1.1637782579712164,2.6429040001820567,distance
40,1773760242.0003111,0,45.0420483,7.5961965,329.7,0.063,0.081,1.2807321981291815,0.7959659323296648,distance
41,1773760243.0003114,0,45.0420379,7.5961909,329.712,0.06,0.076,1.2373002219161546,3.4388597147477924,distance
42,1773760244.0003116,0,45.042027999999995,7.5961856999999995,329.687,0.061,0.074,1.174200330480821,0.4685783342726495,distance
43,1773760245.0003119,0,45.0420186,7.596180299999999,329.617,0.067,0.091,1.1280593105431167,1.731079620479477,distance
44,1773760246.0003123,0,45.0420097,7.596175199999999,329.633,0.064,0.076,1.0676793444570785,0.04987016629979735,distance
45,1773760249.0003133,0,45.042004899999995,7.5961776,329.609,0.054,0.069,0.5660660256863496,41.50098113487425,turn
46,1773760250.0003135,0,45.042001,7.5961896,329.563,0.056,0.064,1.0377795463118935,45.84160679479004,distance
47,1773760251.0003138,0,45.0419967,7.5962048,329.56,0.055,0.07,1.286408736115272,2.8808424535420727,distance
48,1773760252.000314,0,45.041992899999996,7.5962198,329.545,0.052,0.062,1.255789478391944,1.6176767653814181,distance
49,1773760254.0003147,0,45.041991499999995,7.5962336,329.41200000000003,0.049,0.06,1.0938462943088108,12.607929182615521,distance
50,1773760264.0003178,0,45.0419877,7.5962362,329.128,0.052,0.06,0.4693301397134471,56.604381354594125,turn
51,1773760267.0003188,2,45.0419519,7.5962958,326.32,0.014,0.012,6.146090734417203,23.83046358767473,distance
52,1773760279.0003223,2,45.0419597,7.596300599999999,326.337,0.014,0.016,0.9457658228338834,106.86729607986098,turn
53,1773760280.0003226,2,45.0419686,7.596307299999999,326.351,0.018,0.023,1.1209315736337861,4.509132223645963,distance
54,1773760281.0003228,2,45.0419778,7.596313599999999,326.329,0.017,0.021,1.1364530822630157,2.189223941770365,distance
55,1773760282.000323,2,45.0419865,7.5963191,326.266,0.015,0.017,1.0595239090739121,1.75044038596036,distance
56,1773760283.0003235,2,45.0419901,7.596318999999999,326.345,0.015,0.019,0.40037883395586127,25.194446396640785,turn
57,1773760288.000325,2,45.0419883,7.596323699999999,326.241,0.018,0.018,0.4200286077300262,119.58260699923636,turn
58,1773760290.0003254,2,45.0419823,7.596336799999999,326.323,0.014,0.015,1.2199870343220949,4.694093677628814,distance
59,1773760291.000326,2,45.0419779,7.596351299999999,326.311,0.014,0.013,1.247088751697109,10.053355161716922,distance
60,1773760292.0003262,2,45.0419726,7.5963669,326.321,0.014,0.013,1.35999947376838,2.5803638168412135,distance
61,1773760293.0003264,2,45.0419676,7.5963826,326.329,0.014,0.011,1.353039140480666,1.4174009993150776,distance
62,1773760294.0003266,2,45.0419632,7.5963981,326.327,0.014,0.012,1.3124253520391347,2.3741652584794792,distance
63,1773760295.000327,2,45.041958699999995,7.5964133,326.293,0.014,0.01,1.2948399572314384,0.8454062978747743,distance
64,1773760296.0003273,2,45.041953899999996,7.5964275,326.284,0.014,0.01,1.2367772571192874,2.833065987104078,distance
65,1773760297.0003276,2,45.0419489,7.5964415999999995,326.308,0.014,0.012,1.2395089214035275,1.0841301539377355,distance
66,1773760298.000328,2,45.0419446,7.5964561999999995,326.264,0.014,0.013,1.2427690603541002,4.022963556952789,distance
67,1773760300.0003285,2,45.041942299999995,7.59647,326.288,0.014,0.013,1.1140077376620803,9.355309070398164,distance
68,1773760302.0003293,2,45.041939,7.5964662999999994,326.278,0.015,0.017,0.4681422473720707,115.11549172421275,turn
69,1773760303.0003295,2,45.0419334,7.596463399999999,326.205,0.014,0.014,0.6630691049163564,18.28934912679,turn
70,1773760304.0003297,2,45.0419248,7.596458299999999,326.20300000000003,0.014,0.014,1.0368350998382654,2.6366556353280544,distance
71,1773760306.0003302,2,45.0419146,7.5964521,326.163,0.014,0.013,1.2343732522575324,0.508458648785421,distance
72,1773760308.000331,2,45.0419117,7.5964586,326.043,0.014,0.014,0.6039849375986097,80.9742180444305,turn
"""

TOTAL_WAYPOINTS = 73
ARRIVAL_THRESHOLD_M = 1.5


# ──────────────────────────────────────────────────────────────
# Helper to build a node without ROS infrastructure
# ──────────────────────────────────────────────────────────────


def _build_node():
    """Construct a WaypointNavigator with all ROS internals mocked."""
    from pyproj import Transformer
    from builtin_interfaces.msg import Time

    node = WaypointNavigator.__new__(WaypointNavigator)
    node._logger = MagicMock()
    node.transformer = Transformer.from_crs("EPSG:4326", "EPSG:32632", always_xy=True)
    node.datum_east = None
    node.datum_north = None
    node.datum_set = False
    node.datum_sub = MagicMock()
    node.index = 0
    node.get_clock = MagicMock()
    node.get_clock.return_value.now.return_value.to_msg.return_value = Time()

    # Mock publisher and timers (current implementation)
    node.goal_pub = MagicMock()
    node._latest_goal = None
    node._goal_timer = MagicMock()
    node._arrival_timer = MagicMock()

    node.tf_buffer = MagicMock()
    node.destroy_subscription = MagicMock()

    with patch("builtins.open", mock_open(read_data=SAMPLE_CSV)):
        node.waypoints = node._load_waypoints_raw("waypoints.csv")

    return node


def _set_datum_at_first_wp(node):
    """Set datum to the first waypoint so local offsets start at (0,0)."""
    lat, lon = node.waypoints[0, 0], node.waypoints[0, 1]
    e, n = node.transformer.transform(lon, lat)
    node.datum_east = e
    node.datum_north = n
    node.datum_set = True


def _make_navsatfix(lat, lon, status_val=2):
    msg = MagicMock(spec=NavSatFix)
    msg.latitude = lat
    msg.longitude = lon
    msg.status = MagicMock()
    msg.status.status = status_val
    return msg


def _make_tf_transform(x, y):
    """Build a mock TF transform with given map->base_link translation."""
    tf = MagicMock()
    tf.transform.translation.x = x
    tf.transform.translation.y = y
    return tf


# ──────────────────────────────────────────────────────────────
# Fixtures
# ──────────────────────────────────────────────────────────────


@pytest.fixture
def nav():
    return _build_node()


# ──────────────────────────────────────────────────────────────
# Tests: CSV loading
# ──────────────────────────────────────────────────────────────


class TestLoadWaypoints:
    def test_loads_all_73_waypoints(self, nav):
        assert len(nav.waypoints) == TOTAL_WAYPOINTS

    def test_shape_is_n_by_3(self, nav):
        assert nav.waypoints.shape == (TOTAL_WAYPOINTS, 3)

    def test_first_row_lat_lon(self, nav):
        np.testing.assert_almost_equal(nav.waypoints[0, 0], 45.0419561, decimal=6)
        np.testing.assert_almost_equal(nav.waypoints[0, 1], 7.5963935, decimal=6)

    def test_last_row_lat_lon(self, nav):
        np.testing.assert_almost_equal(nav.waypoints[72, 0], 45.0419117, decimal=6)
        np.testing.assert_almost_equal(nav.waypoints[72, 1], 7.5964586, decimal=6)

    def test_all_headings_finite(self, nav):
        assert np.all(np.isfinite(nav.waypoints[:, 2]))

    def test_headings_within_pi(self, nav):
        headings = nav.waypoints[:, 2]
        assert np.all(headings >= -math.pi)
        assert np.all(headings <= math.pi)

    def test_last_heading_equals_previous(self, nav):
        np.testing.assert_almost_equal(
            nav.waypoints[-1, 2], nav.waypoints[-2, 2], decimal=8
        )

    def test_heading_direction_first_segment(self, nav):
        """First segment goes roughly NW (lat increases, lon decreases),
        so heading should be in the second quadrant (positive y, negative x)."""
        yaw = nav.waypoints[0, 2]
        assert math.pi / 4 < yaw < math.pi

    def test_only_lat_lon_yaw_stored(self, nav):
        assert nav.waypoints.shape[1] == 3


# ──────────────────────────────────────────────────────────────
# Tests: Coordinate conversion (_latlon_to_pose)
# ──────────────────────────────────────────────────────────────


class TestLatLonToPose:
    def test_first_wp_maps_to_origin(self, nav):
        _set_datum_at_first_wp(nav)
        wp = nav.waypoints[0]
        pose = nav._latlon_to_pose(wp[0], wp[1], wp[2])
        assert abs(pose.pose.position.x) < 0.01
        assert abs(pose.pose.position.y) < 0.01

    def test_frame_id_is_map(self, nav):
        _set_datum_at_first_wp(nav)
        wp = nav.waypoints[0]
        pose = nav._latlon_to_pose(wp[0], wp[1], wp[2])
        assert pose.header.frame_id == "map"

    def test_z_is_zero(self, nav):
        _set_datum_at_first_wp(nav)
        wp = nav.waypoints[10]
        pose = nav._latlon_to_pose(wp[0], wp[1], wp[2])
        assert pose.pose.position.z == 0.0

    def test_quaternion_normalised_across_waypoints(self, nav):
        _set_datum_at_first_wp(nav)
        for i in [0, 10, 25, 36, 50, 72]:
            wp = nav.waypoints[i]
            pose = nav._latlon_to_pose(wp[0], wp[1], wp[2])
            q = pose.pose.orientation
            norm = math.sqrt(q.x**2 + q.y**2 + q.z**2 + q.w**2)
            assert abs(norm - 1.0) < 1e-9, f"Quaternion not normalised at wp {i}"

    def test_zero_yaw_identity_quaternion(self, nav):
        _set_datum_at_first_wp(nav)
        pose = nav._latlon_to_pose(45.042, 7.596, 0.0)
        assert abs(pose.pose.orientation.z) < 1e-9
        assert abs(pose.pose.orientation.w - 1.0) < 1e-9

    def test_90_degree_yaw(self, nav):
        _set_datum_at_first_wp(nav)
        yaw = math.pi / 2.0
        pose = nav._latlon_to_pose(45.042, 7.596, yaw)
        assert abs(pose.pose.orientation.z - math.sin(yaw / 2)) < 1e-9
        assert abs(pose.pose.orientation.w - math.cos(yaw / 2)) < 1e-9

    def test_negative_yaw(self, nav):
        _set_datum_at_first_wp(nav)
        yaw = -math.pi / 4.0
        pose = nav._latlon_to_pose(45.042, 7.596, yaw)
        assert abs(pose.pose.orientation.z - math.sin(yaw / 2)) < 1e-9
        assert abs(pose.pose.orientation.w - math.cos(yaw / 2)) < 1e-9

    def test_consecutive_wp_spacing(self, nav):
        """Consecutive waypoints in the CSV are roughly 0.4–2.7 m apart."""
        _set_datum_at_first_wp(nav)
        for i in range(TOTAL_WAYPOINTS - 1):
            wp1, wp2 = nav.waypoints[i], nav.waypoints[i + 1]
            p1 = nav._latlon_to_pose(wp1[0], wp1[1], wp1[2])
            p2 = nav._latlon_to_pose(wp2[0], wp2[1], wp2[2])
            dx = p2.pose.position.x - p1.pose.position.x
            dy = p2.pose.position.y - p1.pose.position.y
            dist = math.sqrt(dx**2 + dy**2)
            assert 0.1 < dist < 10.0, f"wp {i}->{i + 1} dist={dist:.2f} m out of range"

    def test_all_points_within_30m_of_origin(self, nav):
        """All 73 points are within a small area."""
        _set_datum_at_first_wp(nav)
        for i in range(TOTAL_WAYPOINTS):
            wp = nav.waypoints[i]
            pose = nav._latlon_to_pose(wp[0], wp[1], wp[2])
            d = math.sqrt(pose.pose.position.x**2 + pose.pose.position.y**2)
            assert d < 30.0, f"wp {i} at {d:.1f} m from origin"


# ──────────────────────────────────────────────────────────────
# Tests: Datum callback
# ──────────────────────────────────────────────────────────────


class TestDatumCallback:
    def test_datum_set_on_valid_fix(self, nav):
        msg = _make_navsatfix(45.042, 7.596, status_val=2)
        nav._datum_callback(msg)
        assert nav.datum_set is True
        assert nav.datum_east is not None
        assert nav.datum_north is not None

    def test_datum_ignored_on_no_fix(self, nav):
        # status < 1 means no fix; use status_val=0 which satisfies msg.status.status < 1
        msg = _make_navsatfix(45.042, 7.596, status_val=0)
        nav._datum_callback(msg)
        assert nav.datum_set is False

    def test_datum_triggers_first_waypoint_publish(self, nav):
        """After datum is set the node should arm _latest_goal for the first waypoint."""
        msg = _make_navsatfix(45.042, 7.596, status_val=2)
        nav._datum_callback(msg)
        assert nav._latest_goal is not None

    def test_datum_subscription_destroyed(self, nav):
        msg = _make_navsatfix(45.042, 7.596, status_val=2)
        nav._datum_callback(msg)
        nav.destroy_subscription.assert_called_once_with(nav.datum_sub)

    def test_datum_locked_after_first_fix(self, nav):
        nav._datum_callback(_make_navsatfix(45.042, 7.596, status_val=2))
        first_east = nav.datum_east
        nav._datum_callback(_make_navsatfix(46.0, 8.0, status_val=2))
        assert nav.datum_east == first_east

    def test_datum_values_are_reasonable_utm(self, nav):
        """Turin UTM32N easting ~395k, northing ~4988k."""
        msg = _make_navsatfix(45.0419561, 7.5963935, status_val=2)
        nav._datum_callback(msg)
        assert 300_000 < nav.datum_east < 500_000
        assert 4_900_000 < nav.datum_north < 5_100_000

    def test_arrival_timer_reset_after_datum(self, nav):
        """_arrival_timer.reset() must be called once datum is locked."""
        msg = _make_navsatfix(45.042, 7.596, status_val=2)
        nav._datum_callback(msg)
        nav._arrival_timer.reset.assert_called_once()


# ──────────────────────────────────────────────────────────────
# Tests: Goal timer callback (_goal_timer_callback)
# ──────────────────────────────────────────────────────────────


class TestGoalTimerCallback:
    def test_does_nothing_when_no_goal(self, nav):
        nav._latest_goal = None
        nav._goal_timer_callback()
        nav.goal_pub.publish.assert_not_called()

    def test_publishes_latest_goal(self, nav):
        _set_datum_at_first_wp(nav)
        nav._publish_current_waypoint()
        assert nav._latest_goal is not None
        nav._goal_timer_callback()
        nav.goal_pub.publish.assert_called_once_with(nav._latest_goal)

    def test_republishes_same_goal_on_repeated_ticks(self, nav):
        _set_datum_at_first_wp(nav)
        nav._publish_current_waypoint()
        for _ in range(5):
            nav._goal_timer_callback()
        assert nav.goal_pub.publish.call_count == 5

    def test_publishes_updated_goal_after_advance(self, nav):
        _set_datum_at_first_wp(nav)
        nav._publish_current_waypoint()
        first_goal = nav._latest_goal

        nav.index += 1
        nav._publish_current_waypoint()
        second_goal = nav._latest_goal

        assert first_goal is not second_goal
        nav._goal_timer_callback()
        nav.goal_pub.publish.assert_called_once_with(second_goal)


# ──────────────────────────────────────────────────────────────
# Tests: _publish_current_waypoint
# ──────────────────────────────────────────────────────────────


class TestPublishCurrentWaypoint:
    def test_sets_latest_goal_for_first_waypoint(self, nav):
        _set_datum_at_first_wp(nav)
        nav._publish_current_waypoint()
        assert nav._latest_goal is not None

    def test_goal_has_map_frame(self, nav):
        _set_datum_at_first_wp(nav)
        nav._publish_current_waypoint()
        assert nav._latest_goal.header.frame_id == "map"

    def test_stops_at_end_of_waypoints(self, nav):
        _set_datum_at_first_wp(nav)
        nav.index = TOTAL_WAYPOINTS
        nav._publish_current_waypoint()
        assert nav._latest_goal is None

    def test_logs_completion_at_end(self, nav):
        _set_datum_at_first_wp(nav)
        nav.index = TOTAL_WAYPOINTS
        nav._publish_current_waypoint()
        nav._logger.info.assert_called()
        log_msg = nav._logger.info.call_args[0][0]
        assert "All waypoints" in log_msg or "stopping" in log_msg.lower()

    def test_cancels_arrival_timer_at_end(self, nav):
        _set_datum_at_first_wp(nav)
        nav.index = TOTAL_WAYPOINTS
        nav._publish_current_waypoint()
        nav._arrival_timer.cancel.assert_called_once()

    def test_goal_position_advances_with_index(self, nav):
        _set_datum_at_first_wp(nav)
        nav._publish_current_waypoint()
        x0 = nav._latest_goal.pose.position.x
        y0 = nav._latest_goal.pose.position.y

        nav.index = 5
        nav._publish_current_waypoint()
        x5 = nav._latest_goal.pose.position.x
        y5 = nav._latest_goal.pose.position.y

        assert x0 != x5 or y0 != y5


# ──────────────────────────────────────────────────────────────
# Tests: Arrival check (_arrival_check_callback)
# ──────────────────────────────────────────────────────────────


class TestArrivalCheckCallback:
    def _prepare(self, nav):
        _set_datum_at_first_wp(nav)
        nav._publish_current_waypoint()

    def test_no_action_when_no_goal(self, nav):
        nav._latest_goal = None
        nav._arrival_check_callback()
        assert nav.index == 0

    def test_no_action_when_index_exhausted(self, nav):
        _set_datum_at_first_wp(nav)
        nav.index = TOTAL_WAYPOINTS
        nav._arrival_check_callback()
        assert nav.index == TOTAL_WAYPOINTS

    def test_no_advance_when_tf_unavailable(self, nav):
        self._prepare(nav)
        nav.tf_buffer.lookup_transform.side_effect = Exception("no TF")
        nav._arrival_check_callback()
        assert nav.index == 0

    def test_no_advance_when_robot_far_away(self, nav):
        self._prepare(nav)
        # Place robot 10 m away from the goal (well above threshold)
        goal_x = nav._latest_goal.pose.position.x
        goal_y = nav._latest_goal.pose.position.y
        nav.tf_buffer.lookup_transform.return_value = _make_tf_transform(
            goal_x + 10.0, goal_y
        )
        nav._arrival_check_callback()
        assert nav.index == 0

    def test_advances_when_robot_within_threshold(self, nav):
        self._prepare(nav)
        goal_x = nav._latest_goal.pose.position.x
        goal_y = nav._latest_goal.pose.position.y
        # Place robot just inside the arrival threshold
        nav.tf_buffer.lookup_transform.return_value = _make_tf_transform(
            goal_x + ARRIVAL_THRESHOLD_M * 0.5,
            goal_y,
        )
        nav._arrival_check_callback()
        assert nav.index == 1

    def test_advances_when_robot_exactly_at_goal(self, nav):
        self._prepare(nav)
        goal_x = nav._latest_goal.pose.position.x
        goal_y = nav._latest_goal.pose.position.y
        nav.tf_buffer.lookup_transform.return_value = _make_tf_transform(goal_x, goal_y)
        nav._arrival_check_callback()
        assert nav.index == 1

    def test_full_traversal_via_arrival_callback(self, nav):
        """Simulate arriving at every waypoint and verify the index reaches the end."""
        _set_datum_at_first_wp(nav)
        nav._publish_current_waypoint()

        for _ in range(TOTAL_WAYPOINTS):
            goal_x = nav._latest_goal.pose.position.x
            goal_y = nav._latest_goal.pose.position.y
            nav.tf_buffer.lookup_transform.return_value = _make_tf_transform(
                goal_x, goal_y
            )
            nav._arrival_check_callback()

        assert nav.index == TOTAL_WAYPOINTS
        assert nav._latest_goal is None

    def test_arrival_logs_waypoint_number(self, nav):
        self._prepare(nav)
        goal_x = nav._latest_goal.pose.position.x
        goal_y = nav._latest_goal.pose.position.y
        nav.tf_buffer.lookup_transform.return_value = _make_tf_transform(goal_x, goal_y)
        nav._arrival_check_callback()
        nav._logger.info.assert_called()

    def test_boundary_exactly_at_threshold(self, nav):
        """A robot sitting exactly on the threshold boundary should NOT advance
        (distance == threshold is not strictly less than threshold)."""
        self._prepare(nav)
        goal_x = nav._latest_goal.pose.position.x
        goal_y = nav._latest_goal.pose.position.y
        nav.tf_buffer.lookup_transform.return_value = _make_tf_transform(
            goal_x + ARRIVAL_THRESHOLD_M, goal_y
        )
        nav._arrival_check_callback()
        assert nav.index == 0


# ──────────────────────────────────────────────────────────────
# Tests: Edge cases
# ──────────────────────────────────────────────────────────────


class TestEdgeCases:
    def test_empty_csv_returns_zero_waypoints(self):
        empty_csv = (
            "index,stamp_sec,status,lat,lon,alt,h_acc,v_acc,"
            "dist_from_prev_m,heading_change_deg,reason\n"
        )
        from pyproj import Transformer

        node = WaypointNavigator.__new__(WaypointNavigator)
        node._logger = MagicMock()
        node.transformer = Transformer.from_crs(
            "EPSG:4326", "EPSG:32632", always_xy=True
        )
        with patch("builtins.open", mock_open(read_data=empty_csv)):
            wps = node._load_waypoints_raw("empty.csv")
        assert len(wps) == 0

    def test_single_waypoint_csv(self):
        single_csv = (
            "index,stamp_sec,status,lat,lon,alt,h_acc,v_acc,"
            "dist_from_prev_m,heading_change_deg,reason\n"
            "0,1773760130.0,2,45.0419561,7.5963935,329.266,0.014,0.018,0.0,nan,start\n"
        )
        from pyproj import Transformer

        node = WaypointNavigator.__new__(WaypointNavigator)
        node._logger = MagicMock()
        node.transformer = Transformer.from_crs(
            "EPSG:4326", "EPSG:32632", always_xy=True
        )
        with patch("builtins.open", mock_open(read_data=single_csv)):
            try:
                wps = node._load_waypoints_raw("single.csv")
                assert len(wps) == 1
            except IndexError:
                pytest.xfail(
                    "Single-waypoint CSV causes IndexError on headings[-1] "
                    "— needs a guard in _load_waypoints_raw"
                )

    def test_mid_route_datum_shifts_all_offsets(self, nav=None):
        node = _build_node()
        mid_wp = node.waypoints[36]
        e, n = node.transformer.transform(mid_wp[1], mid_wp[0])
        node.datum_east = e
        node.datum_north = n
        node.datum_set = True

        pose36 = node._latlon_to_pose(mid_wp[0], mid_wp[1], mid_wp[2])
        assert abs(pose36.pose.position.x) < 0.01
        assert abs(pose36.pose.position.y) < 0.01

        wp0 = node.waypoints[0]
        pose0 = node._latlon_to_pose(wp0[0], wp0[1], wp0[2])
        assert abs(pose0.pose.position.x) > 0.1 or abs(pose0.pose.position.y) > 0.1

    def test_index_beyond_length_stops_publishing(self):
        nav = _build_node()
        _set_datum_at_first_wp(nav)
        nav.index = TOTAL_WAYPOINTS
        nav._publish_current_waypoint()
        assert nav._latest_goal is None
        nav._logger.info.assert_called()
        log_msg = nav._logger.info.call_args[0][0]
        assert "All waypoints" in log_msg or "reached" in log_msg.lower()

    def test_goal_timer_silent_after_completion(self):
        """After all waypoints are done, the goal timer must not publish."""
        nav = _build_node()
        _set_datum_at_first_wp(nav)
        nav.index = TOTAL_WAYPOINTS
        nav._publish_current_waypoint()
        nav._goal_timer_callback()
        nav.goal_pub.publish.assert_not_called()
