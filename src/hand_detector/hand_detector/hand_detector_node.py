#!/usr/bin/env python3
import os
import sys

import cv2
import numpy as np
import rclpy
import torch
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from ultralytics import YOLO


def _to_bgr_image(bridge: CvBridge, msg: Image) -> np.ndarray:
    return bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")


def _empty_mask_like(image: np.ndarray) -> np.ndarray:
    return np.zeros((image.shape[0], image.shape[1]), dtype=np.uint8)


def _resolve_target_class_id(model: YOLO, label: str) -> int:
    names = model.names
    if isinstance(names, dict):
        for class_id, class_name in names.items():
            if str(class_name) == label:
                return int(class_id)
    else:
        for class_id, class_name in enumerate(names):
            if str(class_name) == label:
                return int(class_id)
    raise RuntimeError(f"label '{label}' not found in model names")


def _build_mask_from_result(result, image_shape, target_class_id: int) -> np.ndarray:
    height, width = image_shape[:2]
    mask = np.zeros((height, width), dtype=np.uint8)
    if (
        result.masks is None
        or result.masks.data is None
        or result.boxes is None
        or result.boxes.cls is None
    ):
        return mask

    masks_np = result.masks.data.detach().cpu().numpy()
    classes_np = result.boxes.cls.detach().cpu().numpy().astype(np.int32)
    for index, class_id in enumerate(classes_np):
        if int(class_id) != int(target_class_id):
            continue
        one = masks_np[index]
        resized = cv2.resize(one, (width, height), interpolation=cv2.INTER_LINEAR)
        mask[resized > 0.5] = 255
    return mask


class HandDetectorNode(Node):
    def __init__(self):
        super().__init__("hand_detector")

        self.declare_parameter("model_path", "")
        self.declare_parameter("label", "hand")
        self.declare_parameter("device", "cuda:0")
        self.declare_parameter("confidence", 0.25)
        self.declare_parameter("imgsz", 640)
        self.declare_parameter("max_det", 10)
        self.declare_parameter("retina_masks", False)

        self.model_path = self.get_parameter("model_path").get_parameter_value().string_value
        self.label = self.get_parameter("label").get_parameter_value().string_value
        self.device = self.get_parameter("device").get_parameter_value().string_value
        self.confidence = (
            self.get_parameter("confidence").get_parameter_value().double_value
        )
        self.imgsz = self.get_parameter("imgsz").get_parameter_value().integer_value
        self.max_det = self.get_parameter("max_det").get_parameter_value().integer_value
        self.retina_masks = (
            self.get_parameter("retina_masks").get_parameter_value().bool_value
        )

        self._log_runtime_parameters()
        self._validate_startup_or_exit()
        self.bridge = CvBridge()
        self.model = YOLO(self.model_path, task="segment")
        self.target_class_id = _resolve_target_class_id(self.model, self.label)
        self.get_logger().info(f"resolved.target_class_id={self.target_class_id}")

        self.sub = self.create_subscription(Image, "~/image_raw", self.image_callback, 1)
        self.pub = self.create_publisher(Image, "~/hand_mask", 1)
        self.get_logger().info("Hand detector node started.")

    def _log_runtime_parameters(self):
        self.get_logger().info(f"param.model_path={self.model_path}")
        self.get_logger().info(f"param.label={self.label}")
        self.get_logger().info(f"param.device={self.device}")
        self.get_logger().info(f"param.confidence={self.confidence}")
        self.get_logger().info(f"param.imgsz={self.imgsz}")
        self.get_logger().info(f"param.max_det={self.max_det}")
        self.get_logger().info(f"param.retina_masks={self.retina_masks}")

    def _validate_startup_or_exit(self):
        if not self.model_path:
            self.get_logger().error("model_path is empty.")
            raise RuntimeError("model_path is empty")
        if not os.path.isabs(self.model_path):
            self.get_logger().error("model_path must be an absolute path.")
            raise RuntimeError("model_path must be absolute")
        if not os.path.exists(self.model_path):
            self.get_logger().error("model_path does not exist.")
            raise RuntimeError("model_path does not exist")
        if self.device == "cuda" and not torch.cuda.is_available():
            self.get_logger().error("cuda is not available.")
            raise RuntimeError("cuda not available")

    def image_callback(self, msg: Image):
        try:
            image = _to_bgr_image(self.bridge, msg)
            results = self.model.predict(
                source=image,
                conf=float(self.confidence),
                imgsz=int(self.imgsz),
                max_det=int(self.max_det),
                retina_masks=bool(self.retina_masks),
                device=self.device,
                verbose=False,
            )

            if not results:
                mask = _empty_mask_like(image)
            else:
                # 多目标逻辑: 同一目标类别的全部实例取并集输出
                mask = _build_mask_from_result(
                    results[0], image.shape, self.target_class_id
                )
        except Exception as exc:
            self.get_logger().warning(f"Frame inference failed: {exc}")
            try:
                image = _to_bgr_image(self.bridge, msg)
                mask = _empty_mask_like(image)
            except Exception:
                return

        out = self.bridge.cv2_to_imgmsg(mask, encoding="mono8")
        out.header = msg.header
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = HandDetectorNode()
    except Exception as exc:
        print(f"[hand_detector] Startup failed: {exc}", file=sys.stderr)
        rclpy.shutdown()
        raise

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
