#!/usr/bin/env python3
import importlib
import json
import sys
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from message_filters import ApproximateTimeSynchronizer, Subscriber

_upstream_registered = False


def _register_fastfoundation_upstream():
    global _upstream_registered
    if _upstream_registered:
        return
    current_file = Path(__file__).resolve()
    for parent in current_file.parents:
        for candidate in (
            parent / 'handover_baseline' / 'thirdparty' / 'Fast-FoundationStereo',
        ):
            candidate_str = str(candidate)
            if candidate.is_dir() and candidate_str not in sys.path:
                sys.path.insert(0, candidate_str)
    _upstream_registered = True


def ensure_foundation_stereo_importable():
    _register_fastfoundation_upstream()
    try:
        importlib.import_module('core.foundation_stereo')
    except ModuleNotFoundError as exc:
        raise RuntimeError(
            'Fast-FoundationStereo sources not found. Clone into '
            '`pipeline/external/res/third_party/Fast-FoundationStereo` under the workspace root, '
            'then rebuild. See NVIDIA Fast-FoundationStereo upstream.'
        ) from exc


_STEREO_SYNC_QUEUE_SIZE = 3
_STEREO_SYNC_SLOP_SEC = 0.05
_DEFAULT_HISTORY_DEPTH = 3


def _camera_info_topic_from_image_topic(image_topic: str) -> str:
    """与常见相机驱动一致：与 image 同命名空间下的 camera_info。"""
    t = image_topic.strip().rstrip('/')
    if not t:
        return 'camera_info'
    head, sep, _tail = t.rpartition('/')
    if not sep:
        return f'{t}/camera_info'
    return f'{head}/camera_info'


class FastFoundationNode(Node):
    def __init__(self, node_name):
        super().__init__(node_name)

        self.declare_parameter('cam1_topic', '')
        self.declare_parameter('cam2_topic', '')
        self.declare_parameter('weight_dir', '')
        self.declare_parameter('cam2_camera_info_topic', '')
        self.declare_parameter('device', 'cuda')
        self.declare_parameter('depth_img_topic', '~/depth_img')
        self.declare_parameter('inference_backend', 'engine')

        self.cam1_topic = self.get_parameter('cam1_topic').get_parameter_value().string_value
        self.cam2_topic = self.get_parameter('cam2_topic').get_parameter_value().string_value
        weight_dir = self.get_parameter('weight_dir').get_parameter_value().string_value
        self.depth_img_topic = self.get_parameter('depth_img_topic').get_parameter_value().string_value
        device = self.get_parameter('device').get_parameter_value().string_value
        cam2_ci = self.get_parameter('cam2_camera_info_topic').get_parameter_value().string_value.strip()
        inference_backend = self.get_parameter('inference_backend').get_parameter_value().string_value

        self.bridge = CvBridge()

        self.cam2_camera_info_topic = cam2_ci or _camera_info_topic_from_image_topic(self.cam2_topic)
        self.fx = 0.0
        self.baseline = 0.0
        self._cam_info_ready = False

        from fastfoundation.model_api import api as model_api

        self.predictor = model_api.create_predictor(
            inference_backend=inference_backend,
            weight_dir=weight_dir,
            device=device,
        )

        self.pub_depth_img = self.create_publisher(Image, self.depth_img_topic, _DEFAULT_HISTORY_DEPTH)

        self._sub_cam2_ci = self.create_subscription(
            CameraInfo,
            self.cam2_camera_info_topic,
            self._on_cam2_camera_info,
            _DEFAULT_HISTORY_DEPTH,
        )

        sub_left = Subscriber(self, Image, self.cam1_topic, _DEFAULT_HISTORY_DEPTH)
        sub_right = Subscriber(self, Image, self.cam2_topic, _DEFAULT_HISTORY_DEPTH)
        self._sync = ApproximateTimeSynchronizer(
            [sub_left, sub_right],
            queue_size=_STEREO_SYNC_QUEUE_SIZE,
            slop=_STEREO_SYNC_SLOP_SEC,
        )
        self._sync.registerCallback(self._on_stereo_pair)

        cam2_ci_param_raw = self.get_parameter('cam2_camera_info_topic').get_parameter_value().string_value
        t_cam1 = sub_left.sub.topic_name
        t_cam2 = sub_right.sub.topic_name
        t_ci = self._sub_cam2_ci.topic_name
        t_depth = self.pub_depth_img.topic_name
        self.get_logger().info(
            f'topic_subscribe_cam1_image: {t_cam1}, topic_subscribe_cam2_image: {t_cam2}, '
            f'topic_subscribe_cam2_camera_info: {t_ci}, topic_publish_depth_img: {t_depth}'
        )
        self.get_logger().info(
            f'param_cam1_topic: {json.dumps(self.cam1_topic)}, param_cam2_topic: {json.dumps(self.cam2_topic)}, '
            f'param_cam2_camera_info_topic: {json.dumps(cam2_ci_param_raw)}, '
            f'param_weight_dir: {json.dumps(weight_dir)}, param_device: {json.dumps(device)}, '
            f'param_depth_img_topic: {json.dumps(self.depth_img_topic)}, '
            f'param_inference_backend: {json.dumps(inference_backend)}, '
            f'param_stereo_approx_queue_size: {_STEREO_SYNC_QUEUE_SIZE}, '
            f'param_stereo_approx_slop_sec: {_STEREO_SYNC_SLOP_SEC}, '
            f'param_depth_header_frame_source: {json.dumps("cam1_image")}, '
            f'node_fully_qualified_name: {json.dumps(self.get_fully_qualified_name())}'
        )

    def _on_cam2_camera_info(self, msg: CameraInfo):
        p = msg.p
        if len(p) < 4:
            self.get_logger().warn('CameraInfo.p 长度不足，错误', throttle_duration_sec=5.0)
            return
        fx = float(p[0])
        if fx == 0.0:
            self.get_logger().warn('CameraInfo P[0] fx 为 0，错误', throttle_duration_sec=5.0)
            return
        self.fx = fx
        self.baseline = abs(float(p[3]) / fx)
        if not self._cam_info_ready:
            self._cam_info_ready = True
            self.get_logger().info(
                f'副相机 CameraInfo 已就绪: fx={self.fx:.4f}, baseline={self.baseline:.5f} m'
            )

    def _on_stereo_pair(self, msg_left, msg_right):
        try:
            img1 = self.bridge.imgmsg_to_cv2(msg_left)
            img2 = self.bridge.imgmsg_to_cv2(msg_right)
        except Exception as exc:
            self.get_logger().error(f'cv_bridge convert failed: {exc}')
            return

        img1 = self._ensure_shape(img1)
        img2 = self._ensure_shape(img2)
        if img1 is None or img2 is None:
            self.get_logger().warn('Dropped stereo pair: invalid image shape.', throttle_duration_sec=2.0)
            return

        if not self._cam_info_ready or self.fx == 0.0:
            self.get_logger().warn(
                '等待副相机 CameraInfo，跳过本帧深度。',
                throttle_duration_sec=2.0,
            )
            return

        try:
            disparity = self.predictor.predict(img1, img2)
        except Exception as exc:
            self.get_logger().error(f'predict() failed: {exc}')
            return

        disparity = np.clip(disparity, 0.1, None)
        depth = (self.fx * self.baseline) / disparity

        out = self.bridge.cv2_to_imgmsg(depth.astype(np.float32), '32FC1')
        out.header = msg_left.header
        self.pub_depth_img.publish(out)

    def _ensure_shape(self, img):
        h, w = img.shape[:2]
        if h > 480 or w > 640:
            self.get_logger().error(
                f'Image too large {w}x{h}, max 640x480.', throttle_duration_sec=2.0
            )
            return None
        if len(img.shape) == 2:
            return cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)
        if len(img.shape) == 3 and img.shape[2] >= 3:
            return img[:, :, :3]
        self.get_logger().error('Unsupported channel layout.', throttle_duration_sec=2.0)
        return None


def main(args=None):
    rclpy.init(args=args)
    node_name = 'fast_foundation_node'
    node = FastFoundationNode(node_name)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
