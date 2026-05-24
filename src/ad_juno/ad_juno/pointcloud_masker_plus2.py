#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Image, CameraInfo
from cv_bridge import CvBridge
import threading
import numpy as np
import cv2
from shared_objects.ROS_utils import Topics
from rclpy.qos import (
    qos_profile_sensor_data,
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy,
)

# ZED optical frame: X=right, Y=down, Z=forward
# To spoof a point as "tall", set Y negative (upward in optical frame)
SPOOF_AXIS = "y"
SPOOF_VALUE = -1.2  # 1.2m above camera in optical frame
BOUNDARY_KERNEL_PX = 3
BOUNDARY_ITERATIONS = 2  # erode inward — wall inside road edge, not outside
CAMERA_HEIGHT_M = 0.90  # measure from your actual mount to ground


class PointCloudMaskerNode(Node):
    def __init__(self):
        super().__init__("pc_masker_node")
        self.bridge = CvBridge()
        topics = Topics()

        qos_be = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Latest segmentation mask shared between callbacks
        self.latest_mask = None
        self.masking_lock = threading.Lock()

        # Ground plane cache — built once when camera_info arrives
        # Shape (H, W) float32: per-pixel synthetic Z to ground intersection.
        # NaN for pixels at or above the horizon.
        self._ground_depth_cache = None
        self._cache_shape = None  # (H, W) of the cache as built
        self.fx = self.fy = self.cx = self.cy = None

        # Subscriptions
        self.create_subscription(
            Image,
            topics.topic_names["segmented_image"],
            self.mask_callback,
            qos_be,
        )
        self.create_subscription(
            PointCloud2,
            "/zed/zed_node/point_cloud/cloud_registered",
            self.cloud_callback,
            qos_profile_sensor_data,
        )
        # CameraInfo is latched by the ZED wrapper — depth=1 RELIABLE is enough
        self.create_subscription(
            CameraInfo,
            "/zed/zed_node/rgb/camera_info",
            self._camera_info_cb,
            qos_be,
        )

        self.pub_pc = self.create_publisher(
            PointCloud2, "/zed/zed_node/point_cloud/road_boundaries", 1
        )

        self.get_logger().info(
            f"Boundary masker ready — spoofing axis '{SPOOF_AXIS}' = {SPOOF_VALUE} m, "
            f"camera height = {CAMERA_HEIGHT_M} m. Waiting for camera_info..."
        )

    # ------------------------------------------------------------------
    # Camera intrinsics — called once (ZED wrapper latches camera_info)
    # ------------------------------------------------------------------
    def _camera_info_cb(self, msg: CameraInfo):
        if self.fx is not None:
            return  # already built, ignore repeats

        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]

        self._build_ground_cache(msg.height, msg.width)
        self.get_logger().info(
            f"Intrinsics received: fx={self.fx:.1f} fy={self.fy:.1f} "
            f"cx={self.cx:.1f} cy={self.cy:.1f}. Ground cache built for "
            f"{msg.width}x{msg.height}."
        )

    def _build_ground_cache(self, h: int, w: int):
        """
        Precompute for every pixel the Z distance to the ground plane.

        Camera optical frame: X right, Y down, Z forward.
        Ray through pixel (u, v): d = [(u-cx)/fx, (v-cy)/fy, 1.0]
        Ground plane: Y = CAMERA_HEIGHT_M  (ground is below camera)
        Intersection: t = CAMERA_HEIGHT_M / ray_y  →  Z_ground = t * 1.0 = t

        Pixels on or above the horizon have ray_y <= 0 → stored as NaN.
        These pixels will never be road boundary pixels so they're safe to skip.
        """
        rows = np.arange(h, dtype=np.float32)
        cols = np.arange(w, dtype=np.float32)
        col_grid, row_grid = np.meshgrid(cols, rows)

        ray_y = (row_grid - self.cy) / self.fy

        with np.errstate(divide="ignore", invalid="ignore"):
            t = np.where(ray_y > 0.01, CAMERA_HEIGHT_M / ray_y, np.nan)

        self._ground_depth_cache = t.astype(np.float32)
        self._cache_shape = (h, w)

        valid_count = np.isfinite(self._ground_depth_cache).sum()
        self.get_logger().info(
            f"Ground cache: {valid_count}/{h * w} pixels have valid ground intersection."
        )

    # ------------------------------------------------------------------
    # Segmentation mask callback (~3.3 Hz)
    # ------------------------------------------------------------------
    def mask_callback(self, mask_msg: Image):
        cv_mask = self.bridge.imgmsg_to_cv2(mask_msg, desired_encoding="mono8")

        # Resize to point cloud resolution BEFORE morphology so the kernel
        # operates in point-cloud pixel units, not image pixel units
        TARGET_H, TARGET_W = 256, 448
        if cv_mask.shape != (TARGET_H, TARGET_W):
            cv_mask = cv2.resize(
                cv_mask, (TARGET_W, TARGET_H), interpolation=cv2.INTER_NEAREST
            )

        kernel = np.ones((BOUNDARY_KERNEL_PX, BOUNDARY_KERNEL_PX), np.uint8)
        road_eroded = cv2.erode(cv_mask, kernel, iterations=BOUNDARY_ITERATIONS)

        # Boundary ring: pixels that are road in the original but disappear
        # after erosion — i.e. the inner edge of the segmented road region.
        new_mask = {
            "boundary_pixels": (cv_mask == 255) & (road_eroded == 0),
            "shape": cv_mask.shape,  # always (TARGET_H, TARGET_W) after resize
        }
        with self.masking_lock:
            self.latest_mask = new_mask

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    def _get_axis_offset(self, pc_msg: PointCloud2) -> int | None:
        offset = next((f.offset for f in pc_msg.fields if f.name == SPOOF_AXIS), None)
        if offset is None:
            self.get_logger().error(
                f"Field '{SPOOF_AXIS}' not found in point cloud. "
                f"Available: {[f.name for f in pc_msg.fields]}"
            )
        return offset

    def _get_ground_cache_for(self, h: int, w: int) -> np.ndarray:
        """
        Return the ground depth cache resized to (h, w) if needed.
        The cache is built from RGB camera_info which may be higher resolution
        than the point cloud (1280x720 vs 448x256). We resize lazily and reuse.
        """
        if self._ground_depth_cache is None:
            return None
        if self._cache_shape == (h, w):
            return self._ground_depth_cache
        # Resize once and cache the result for the point cloud resolution
        resized = cv2.resize(
            self._ground_depth_cache,
            (w, h),
            interpolation=cv2.INTER_LINEAR,
        )
        # Store resized version so we don't repeat this every frame
        self._ground_depth_cache = resized
        self._cache_shape = (h, w)
        self.get_logger().info(
            f"Ground cache resized to {w}x{h} (point cloud resolution)."
        )
        return self._ground_depth_cache

    # ------------------------------------------------------------------
    # Point cloud callback (10 Hz)
    # ------------------------------------------------------------------
    def cloud_callback(self, pc_msg: PointCloud2):
        with self.masking_lock:
            latest = self.latest_mask
        if latest is None:
            return

        if pc_msg.height == 1:
            self.get_logger().warn_once("Unordered point cloud — skipping")
            return

        axis_offset = self._get_axis_offset(pc_msg)
        if axis_offset is None:
            return

        boundary_pixels = latest["boundary_pixels"]
        mask_shape = latest["shape"]

        # mask_callback already resizes to TARGET (256, 448) so this branch
        # should never fire in normal operation, but kept as a safety net
        if mask_shape != (pc_msg.height, pc_msg.width):
            boundary_pixels = cv2.resize(
                boundary_pixels.astype(np.uint8),
                (pc_msg.width, pc_msg.height),
                interpolation=cv2.INTER_NEAREST,
            ).astype(bool)

        cloud_data = np.frombuffer(pc_msg.data, dtype=np.uint8).copy()

        axis_view = np.ndarray(
            shape=(pc_msg.height, pc_msg.width),
            dtype=np.float32,
            buffer=cloud_data,
            offset=axis_offset,
            strides=(pc_msg.row_step, pc_msg.point_step),
        )

        # --- Ground plane NaN fill ---
        # For boundary pixels where the ZED returned no valid depth (NaN/Inf),
        # substitute the synthetic depth from the precomputed ground plane
        # intersection. This ensures the spoof fires even at road edges where
        # the ZED returns invalid readings due to low texture or grazing angle.
        # The Y channel gets overwritten to SPOOF_VALUE anyway — we only need
        # a finite value here so the isfinite gate below passes.
        invalid_boundary = boundary_pixels & ~np.isfinite(axis_view)
        if invalid_boundary.any():
            cache = self._get_ground_cache_for(pc_msg.height, pc_msg.width)
            if cache is not None:
                # Only fill pixels where the ground cache itself is valid
                # (i.e. below the horizon — ray_y > 0.01)
                fillable = invalid_boundary & np.isfinite(cache)
                if fillable.any():
                    axis_view[fillable] = cache[fillable]
                    self.get_logger().debug(
                        f"Ground-filled {fillable.sum()} NaN boundary pixels."
                    )

        # --- Spoof ---
        # All boundary pixels that now have a finite depth value get their
        # Y coordinate overwritten to SPOOF_VALUE, making them appear as a
        # tall obstacle to the costmap's height filter.
        valid = np.isfinite(axis_view)
        spoof = boundary_pixels & valid
        axis_view[spoof] = SPOOF_VALUE

        # Publish modified cloud with same header/metadata as input
        out_msg = PointCloud2()
        out_msg.header = pc_msg.header
        out_msg.height = pc_msg.height
        out_msg.width = pc_msg.width
        out_msg.fields = pc_msg.fields
        out_msg.is_bigendian = pc_msg.is_bigendian
        out_msg.point_step = pc_msg.point_step
        out_msg.row_step = pc_msg.row_step
        out_msg.is_dense = pc_msg.is_dense
        out_msg.data = cloud_data.tobytes()

        self.pub_pc.publish(out_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudMaskerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
