import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError
import cv2
import time
from ultralytics import YOLO


class DetectPerson(Node):

    def __init__(self):
        super().__init__('detect_person')

        self.declare_parameter("image_topic", "/person_tracker/input_image")
        self.declare_parameter("detected_person_topic", "/person_tracker/detected_person")
        self.declare_parameter("debug_image_topic", "/person_tracker/debug_image")
        self.declare_parameter("model_name", "yolov8n.pt")
        self.declare_parameter("min_confidence", 0.45)
        self.declare_parameter("score_confidence_weight", 1.0)
        self.declare_parameter("score_area_weight", 1.0)
        self.declare_parameter("score_center_weight", 0.7)
        self.declare_parameter("inference_rate_hz", 2.0)

        image_topic = self.get_parameter("image_topic").value
        detected_person_topic = self.get_parameter("detected_person_topic").value
        debug_image_topic = self.get_parameter("debug_image_topic").value
        model_name = self.get_parameter("model_name").value

        self.min_confidence = float(self.get_parameter("min_confidence").value)
        self.score_confidence_weight = float(self.get_parameter("score_confidence_weight").value)
        self.score_area_weight = float(self.get_parameter("score_area_weight").value)
        self.score_center_weight = float(self.get_parameter("score_center_weight").value)
        inference_rate_hz = float(self.get_parameter("inference_rate_hz").value)
        self.inference_period_sec = 1.0 / max(inference_rate_hz, 0.1)
        self.last_inference_time = 0.0

        image_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.image_sub = self.create_subscription(Image, image_topic, self.callback, image_qos)
        self.image_out_pub = self.create_publisher(Image, debug_image_topic, 1)
        self.person_pub = self.create_publisher(Point, detected_person_topic, 1)

        self.bridge = CvBridge()
        self.model = YOLO(model_name)
        self.get_logger().info(f"Detecting people from {image_topic}")
        self.get_logger().info(f"Publishing target to {detected_person_topic}")

    def callback(self, data):
        now = time.monotonic()
        if now - self.last_inference_time < self.inference_period_sec:
            return
        self.last_inference_time = now

        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as exc:
            self.get_logger().warning(f"Could not convert image: {exc}")
            return

        height, width = cv_image.shape[:2]
        image_area = float(width * height)
        debug_image = cv_image.copy()

        results = self.model.predict(
            source=cv_image,
            stream=False,
            show=False,
            classes=[0],
            conf=self.min_confidence,
            verbose=False,
        )

        if not results or results[0].boxes is None or len(results[0].boxes) == 0:
            self.publish_debug_image(debug_image, data.header)
            return

        boxes = results[0].boxes.data.cpu().numpy()
        selected_box = self.select_target_box(boxes, width, height, image_area)
        if selected_box is None:
            self.publish_debug_image(debug_image, data.header)
            return

        x1, y1, x2, y2, confidence, _class_id = selected_box
        center_x = 0.5 * (x1 + x2)
        center_y = 0.5 * (y1 + y2)
        bbox_height = max(0.0, y2 - y1)

        point_out = Point()
        point_out.x = (center_x - 0.5 * width) / (0.5 * width)
        point_out.y = (center_y - 0.5 * height) / (0.5 * height)
        point_out.z = bbox_height / float(height)
        self.person_pub.publish(point_out)

        cv2.rectangle(
            debug_image,
            (int(x1), int(y1)),
            (int(x2), int(y2)),
            (0, 255, 0),
            2,
        )
        cv2.circle(debug_image, (int(center_x), int(center_y)), 5, (0, 0, 255), -1)
        cv2.line(debug_image, (width // 2, 0), (width // 2, height), (255, 0, 0), 1)
        cv2.putText(
            debug_image,
            f"person {confidence:.2f}",
            (int(x1), max(20, int(y1) - 8)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 0),
            2,
        )
        self.publish_debug_image(debug_image, data.header)

    def select_target_box(self, boxes, width, height, image_area):
        best_box = None
        best_score = None

        for box in boxes:
            x1, y1, x2, y2, confidence, _class_id = box
            if confidence < self.min_confidence:
                continue

            bbox_area = max(0.0, x2 - x1) * max(0.0, y2 - y1)
            center_x = 0.5 * (x1 + x2)
            normalized_center_error = abs((center_x - 0.5 * width) / (0.5 * width))
            normalized_area = bbox_area / image_area

            score = (
                self.score_confidence_weight * confidence
                + self.score_area_weight * normalized_area
                - self.score_center_weight * normalized_center_error
            )

            if best_score is None or score > best_score:
                best_score = score
                best_box = box

        return best_box

    def publish_debug_image(self, cv_image, header):
        try:
            img_to_pub = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
        except CvBridgeError as exc:
            self.get_logger().warning(f"Could not convert debug image: {exc}")
            return

        img_to_pub.header = header
        self.image_out_pub.publish(img_to_pub)


def main(args=None):

    rclpy.init(args=args)

    detect_person = DetectPerson()
    while rclpy.ok():
        rclpy.spin(detect_person)
        

    detect_person.destroy_node()
    rclpy.shutdown()
