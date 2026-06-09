#!/usr/bin/env python3
"""ROS2 节点：D435 + YOLO + 点云拟合 + piper_ros 控制。

节点职责：
- 从 D435 获取彩色图和深度图。
- 使用 YOLO 检测目标。
- 调用 catch 算法包估计抓取点。
- 订阅 /end_pose_stamped，更新腕部相机的 T_base_camera。
- 根据 execute 参数决定是否向 /pos_cmd 发布控制命令。
"""

from __future__ import annotations

from pathlib import Path
import math
import sys
import time
from typing import Any

DEFAULT_CATCH_PATH = Path("/home/artorias/work/catch")
if DEFAULT_CATCH_PATH.exists():
    sys.path.insert(0, str(DEFAULT_CATCH_PATH))

import rclpy
from geometry_msgs.msg import Pose, PoseStamped
from piper_msgs.msg import PosCmd
from piper_msgs.srv import Enable
from rclpy.node import Node
from std_msgs.msg import Bool

from catch.calibration import load_handeye_calibration
from catch.d435 import depth_at_center, realsense_intrinsics
from catch.geometry import Pose3D, RigidTransform, box_center_from_xyxy, deproject_pixel_to_point
from catch.gripper import MechanicalClampPlanner
from catch.models import KfsDetection, KfsType, VisualTarget
from catch.perception import classify_name
from catch.point_cloud import (
    PlaneFitConfig,
    PlaneFitError,
    estimate_top_grasp_from_any_visible_face,
)
from catch.visualization import draw_target_overlay


class CatchNode(Node):
    """可由 ros2 run/launch 启动的抓取视觉控制节点。"""

    def __init__(self) -> None:
        super().__init__("catch_node")
        self._declare_parameters()
        self._load_parameters()

        self.cv2, self.np, self.rs, yolo_cls = self._load_optional_modules()
        self.model = yolo_cls(str(self.model_path))
        self.gripper = MechanicalClampPlanner()
        self.plane_config = PlaneFitConfig()

        self.pos_pub = self.create_publisher(PosCmd, self.pos_cmd_topic, 1)
        self.moveit_target_pub = self.create_publisher(PoseStamped, self.moveit_target_pose_topic, 1)
        self.enable_pub = self.create_publisher(Bool, self.enable_flag_topic, 1)
        self.enable_client = self.create_client(Enable, self.enable_service)
        self.create_subscription(PoseStamped, self.end_pose_topic, self._pose_stamped_cb, 10)
        self.create_subscription(Pose, self.end_pose_fallback_topic, self._pose_cb, 10)

        self.camera_mode, self.camera_mount_transform = self._load_camera_mount_transform()
        self.t_base_camera = self.camera_mount_transform
        self.last_end_transform: RigidTransform | None = None
        self.last_graph_warn_time = 0.0

        self.pipeline = self.rs.pipeline()
        cfg = self.rs.config()
        cfg.enable_stream(self.rs.stream.color, self.color_width, self.color_height, self.rs.format.bgr8, 30)
        cfg.enable_stream(self.rs.stream.depth, self.depth_width, self.depth_height, self.rs.format.z16, 30)
        profile = self.pipeline.start(cfg)
        self.align = self.rs.align(self.rs.stream.color)
        color_profile = profile.get_stream(self.rs.stream.color).as_video_stream_profile()
        self.intrinsics = realsense_intrinsics(color_profile.get_intrinsics())

        if self.show_window:
            self.cv2.namedWindow(self.window_name, self.cv2.WINDOW_NORMAL)

        if self.enable_arm:
            self._request_enable(True)

        self.timer = self.create_timer(max(0.001, 1.0 / self.rate_hz), self._tick)
        self.get_logger().info(
            f"catch_node started: execute={self.execute}, target_class={self.target_class}, "
            f"pos_cmd_topic={self.pos_cmd_topic}, end_pose_topic={self.end_pose_topic}"
        )
        self.initial_arm_pose()

    def destroy_node(self) -> bool:
        self.pipeline.stop()
        if self.show_window:
            self.cv2.destroyAllWindows()
        return super().destroy_node()

    def _declare_parameters(self) -> None:
        self.declare_parameter("model_path", "/home/artorias/work/vision_model/best.pt")
        self.declare_parameter("target_class", "red")
        self.declare_parameter("conf", 0.75)
        self.declare_parameter("rate_hz", 10.0)
        self.declare_parameter("execute", False)
        self.declare_parameter("enable_arm", False)
        self.declare_parameter("close_gripper", False)
        self.declare_parameter("plane_fit", True)
        self.declare_parameter("show_window", True)
        self.declare_parameter("window_name", "catch ROS D435 target center")
        self.declare_parameter("calib_path", "/home/artorias/piperx_d435_1.calib")
        self.declare_parameter("no_calib", False)
        self.declare_parameter("invert_calib", False)
        self.declare_parameter("cmd_roll", 0.0)
        self.declare_parameter("cmd_pitch", 0.0)
        self.declare_parameter("cmd_yaw", 0.0)
        self.declare_parameter("top_normal_threshold", 0.65)
        self.declare_parameter("side_normal_threshold", 0.45)
        self.declare_parameter("pos_cmd_topic", "/pos_cmd")
        self.declare_parameter("moveit_target_pose_topic", "/catch/top_pregrasp_pose")
        self.declare_parameter("publish_moveit_target", True)
        self.declare_parameter("enable_flag_topic", "/enable_flag")
        self.declare_parameter("enable_service", "/enable_srv")
        self.declare_parameter("end_pose_topic", "/end_pose_stamped")
        self.declare_parameter("end_pose_fallback_topic", "/end_pose")
        self.declare_parameter("color_width", 640)
        self.declare_parameter("color_height", 480)
        self.declare_parameter("depth_width", 640)
        self.declare_parameter("depth_height", 480)

    def _load_parameters(self) -> None:
        self.model_path = Path(self.get_parameter("model_path").value).expanduser()
        if not self.model_path.exists():
            raise FileNotFoundError(f"YOLO model does not exist: {self.model_path}")

        self.target_class = str(self.get_parameter("target_class").value)
        self.conf = float(self.get_parameter("conf").value)
        self.rate_hz = float(self.get_parameter("rate_hz").value)
        self.execute = bool(self.get_parameter("execute").value)
        self.enable_arm = bool(self.get_parameter("enable_arm").value)
        self.close_gripper = bool(self.get_parameter("close_gripper").value)
        self.plane_fit = bool(self.get_parameter("plane_fit").value)
        self.show_window = bool(self.get_parameter("show_window").value)
        self.window_name = str(self.get_parameter("window_name").value)
        self.calib_path = Path(str(self.get_parameter("calib_path").value)).expanduser()
        self.no_calib = bool(self.get_parameter("no_calib").value)
        self.invert_calib = bool(self.get_parameter("invert_calib").value)
        self.cmd_roll = float(self.get_parameter("cmd_roll").value)
        self.cmd_pitch = float(self.get_parameter("cmd_pitch").value)
        self.cmd_yaw = float(self.get_parameter("cmd_yaw").value)
        self.top_normal_threshold = float(self.get_parameter("top_normal_threshold").value)
        self.side_normal_threshold = float(self.get_parameter("side_normal_threshold").value)
        self.pos_cmd_topic = str(self.get_parameter("pos_cmd_topic").value)
        self.moveit_target_pose_topic = str(self.get_parameter("moveit_target_pose_topic").value)
        self.publish_moveit_target = bool(self.get_parameter("publish_moveit_target").value)
        self.enable_flag_topic = str(self.get_parameter("enable_flag_topic").value)
        self.enable_service = str(self.get_parameter("enable_service").value)
        self.end_pose_topic = str(self.get_parameter("end_pose_topic").value)
        self.end_pose_fallback_topic = str(self.get_parameter("end_pose_fallback_topic").value)
        self.color_width = int(self.get_parameter("color_width").value)
        self.color_height = int(self.get_parameter("color_height").value)
        self.depth_width = int(self.get_parameter("depth_width").value)
        self.depth_height = int(self.get_parameter("depth_height").value)

    def _load_optional_modules(self) -> tuple[Any, Any, Any, Any]:
        try:
            import cv2
            import numpy as np
            import pyrealsense2 as rs
            from ultralytics import YOLO
        except ImportError as exc:
            raise RuntimeError(
                "缺少依赖：需要 numpy、pyrealsense2、opencv-python、ultralytics。"
            ) from exc
        return cv2, np, rs, YOLO

    def _load_camera_mount_transform(self) -> tuple[str, RigidTransform]:
        if self.no_calib or not self.calib_path:
            self.get_logger().warn("未使用手眼标定：T_base_camera = identity，仅适合视觉测试")
            return "identity", RigidTransform.identity()

        calib = load_handeye_calibration(self.calib_path)
        transform = calib.transform.inverse() if self.invert_calib else calib.transform
        if calib.calibration_type == "eye_in_hand":
            self.get_logger().info(
                f"loaded eye_in_hand calib: {self.calib_path}; "
                "T_base_camera = T_base_effector_from_ros * T_effector_camera"
            )
            return "eye_in_hand", transform

        self.get_logger().info(f"loaded fixed-base calib as T_base_camera: {self.calib_path}")
        return calib.calibration_type or "fixed_base", transform

    def _tick(self) -> None:
        self._update_graph_diagnostics()
        if self.camera_mode == "eye_in_hand" and self.last_end_transform is not None:
            self.t_base_camera = self.last_end_transform.compose(self.camera_mount_transform)

        frames = self.align.process(self.pipeline.wait_for_frames())
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()
        if not color_frame or not depth_frame:
            return

        image = self.np.asanyarray(color_frame.get_data())
        vis_image = image.copy()
        result = self.model.predict(image, conf=self.conf, verbose=False)[0]

        best: VisualTarget | None = None
        visual_targets: list[VisualTarget] = []
        for box in result.boxes:
            cls_id = int(box.cls[0])
            cls_name = result.names[cls_id]
            kfs_type = classify_name(cls_name)
            x1, y1, x2, y2 = [float(value) for value in box.xyxy[0]]
            u, v = box_center_from_xyxy(x1, y1, x2, y2)
            depth_m = depth_at_center(depth_frame, int(u), int(v))
            if depth_m <= 0.0:
                self._draw_no_depth(vis_image, cls_name, float(box.conf[0]), x1, y1, x2, y2, u, v)
                continue

            normal_to_camera = None
            inlier_ratio = None
            if self.plane_fit:
                try:
                    estimate = estimate_top_grasp_from_any_visible_face(
                        depth_frame=depth_frame,
                        xyxy=(x1, y1, x2, y2),
                        intrinsics=self.intrinsics,
                        t_base_camera=self.t_base_camera,
                        config=self.plane_config,
                        top_normal_threshold=self.top_normal_threshold,
                        side_normal_threshold=self.side_normal_threshold,
                    )
                    pose = Pose3D(
                        x=estimate.top_pre_grasp_base[0],
                        y=estimate.top_pre_grasp_base[1],
                        z=estimate.top_pre_grasp_base[2],
                    )
                    normal_to_camera = estimate.visible_normal_camera
                    inlier_ratio = estimate.inlier_ratio
                except PlaneFitError as exc:
                    self.get_logger().warn(f"平面拟合失败，退回中心点深度: {exc}")
                    pose = self._make_pose_from_center(u, v, depth_m)
            else:
                pose = self._make_pose_from_center(u, v, depth_m)

            detection = KfsDetection(
                object_id=f"{cls_name}-{time.time():.3f}",
                kfs_type=kfs_type,
                confidence=float(box.conf[0]),
                pose=pose,
            )
            target = VisualTarget(
                detection=detection,
                xyxy=(int(x1), int(y1), int(x2), int(y2)),
                center_uv=(int(u), int(v)),
                depth_m=depth_m,
                cls_name=cls_name,
                normal_to_camera=normal_to_camera,
                inlier_ratio=inlier_ratio,
            )
            visual_targets.append(target)

            is_target_class = cls_name == self.target_class or kfs_type == KfsType.R2
            if is_target_class and (best is None or detection.confidence > best.detection.confidence):
                best = target

        for target in visual_targets:
            draw_target_overlay(self.cv2, vis_image, target, selected=target is best)

        if best:
            self._handle_best_target(best)
        else:
            self.get_logger().info("未发现可靠目标", throttle_duration_sec=1.0)

        if self.show_window:
            self.cv2.putText(
                vis_image,
                "green=selected, gray=other, orange=no depth, q/ESC=quit",
                (10, 24),
                self.cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (255, 255, 255),
                2,
            )
            self.cv2.imshow(self.window_name, vis_image)
            key = self.cv2.waitKey(1) & 0xFF
            if key in (27, ord("q")):
                rclpy.shutdown()

    def _make_pose_from_center(self, u: float, v: float, depth_m: float) -> Pose3D:
        point_camera = deproject_pixel_to_point(u, v, depth_m, self.intrinsics)
        x, y, z = self.t_base_camera.apply(point_camera)
        return Pose3D(x=x, y=y, z=z)

    def _handle_best_target(self, best: VisualTarget) -> None:
        setpoint = self.gripper.grasp_setpoint(best.detection)
        pose = best.detection.pose
        self.get_logger().info(
            f"target cls={best.cls_name} conf={best.detection.confidence:.2f} "
            f"center={best.center_uv} depth={best.depth_m:.3f}m "
            f"pose=({pose.x:.3f}, {pose.y:.3f}, {pose.z:.3f})"
        )

        if self.publish_moveit_target:
            target_pose = PoseStamped()
            target_pose.header.frame_id = "base_link"
            target_pose.header.stamp = self.get_clock().now().to_msg()
            target_pose.pose.position.x = float(pose.x)
            target_pose.pose.position.y = float(pose.y)
            target_pose.pose.position.z = float(pose.z)
            qx, qy, qz, qw = _quaternion_from_rpy(self.cmd_roll, self.cmd_pitch, self.cmd_yaw)
            target_pose.pose.orientation.x = qx
            target_pose.pose.orientation.y = qy
            target_pose.pose.orientation.z = qz
            target_pose.pose.orientation.w = qw
            self.moveit_target_pub.publish(target_pose)

        msg = PosCmd()
        msg.x = float(pose.x)
        msg.y = float(pose.y)
        msg.z = float(pose.z)
        msg.roll = self.cmd_roll
        msg.pitch = self.cmd_pitch
        msg.yaw = self.cmd_yaw
        msg.gripper = float(setpoint.opening_m if self.close_gripper else 0.0)
        msg.mode1 = 0
        msg.mode2 = 0

        if self.execute:
            self.pos_pub.publish(msg)
        else:
            self.get_logger().info(
                "DRY-RUN /pos_cmd "
                f"x={msg.x:.4f} y={msg.y:.4f} z={msg.z:.4f} "
                f"roll={msg.roll:.4f} pitch={msg.pitch:.4f} yaw={msg.yaw:.4f} "
                f"gripper={msg.gripper:.4f}",
                throttle_duration_sec=0.5,
            )
    def initial_arm_pose(self) -> None:
        msg = PosCmd()
        msg.x = 0.2
        msg.y = 0.0
        msg.z = 0.32
        msg.roll = 0.0
        msg.pitch = 1.5708
        msg.yaw = 0.0
        msg.gripper = 0.0
        msg.mode1 = 0
        msg.mode2 = 0
        self.pos_pub.publish(msg)


    def _request_enable(self, enabled: bool) -> None:
        flag = Bool()
        flag.data = bool(enabled)
        self.enable_pub.publish(flag)

        if self.enable_client.wait_for_service(timeout_sec=0.5):
            req = Enable.Request()
            req.enable_request = bool(enabled)
            self.enable_client.call_async(req)
            self.get_logger().info(f"requested enable={enabled} through {self.enable_service}")
        else:
            self.get_logger().warn(f"{self.enable_service} 不可用，已先发布 {self.enable_flag_topic}")

    def _update_graph_diagnostics(self) -> None:
        now = time.time()
        if now - self.last_graph_warn_time < 2.0:
            return
        self.last_graph_warn_time = now

        if self.pos_pub.get_subscription_count() == 0:
            self.get_logger().warn(f"{self.pos_cmd_topic} 当前没有订阅者，请确认 piper_single_ctrl 已启动")
        if self.camera_mode == "eye_in_hand" and self.last_end_transform is None:
            self.get_logger().warn(f"尚未收到 {self.end_pose_topic}，腕部相机坐标暂不可信")

    def _pose_stamped_cb(self, msg: PoseStamped) -> None:
        self._update_end_pose(msg.pose)

    def _pose_cb(self, msg: Pose) -> None:
        self._update_end_pose(msg)

    def _update_end_pose(self, pose: Pose) -> None:
        self.last_end_transform = RigidTransform.from_translation_quaternion(
            (float(pose.position.x), float(pose.position.y), float(pose.position.z)),
            (
                float(pose.orientation.x),
                float(pose.orientation.y),
                float(pose.orientation.z),
                float(pose.orientation.w),
            ),
        )

    def _draw_no_depth(
        self,
        vis_image: Any,
        cls_name: str,
        confidence: float,
        x1: float,
        y1: float,
        x2: float,
        y2: float,
        u: float,
        v: float,
    ) -> None:
        self.cv2.rectangle(vis_image, (int(x1), int(y1)), (int(x2), int(y2)), (0, 165, 255), 1)
        self.cv2.drawMarker(
            vis_image,
            (int(u), int(v)),
            (0, 165, 255),
            markerType=self.cv2.MARKER_CROSS,
            markerSize=16,
            thickness=2,
        )
        self.cv2.putText(
            vis_image,
            f"{cls_name} {confidence:.2f} no depth",
            (int(x1), max(20, int(y1) - 8)),
            self.cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 165, 255),
            2,
        )


def _quaternion_from_rpy(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    """把 RPY 弧度转成四元数，顺序为 x, y, z, w。"""

    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = CatchNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
