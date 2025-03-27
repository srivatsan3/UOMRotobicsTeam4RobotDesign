import rclpy
from rclpy.node import Node
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import argparse
import os
import numpy as np
import onnxruntime as ort
import time
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import Header
from yolox_pose_estimator.object_pose_utils_onnx import get_cuboid_corner, get_camera_matrix, get_class_names, \
    draw_obj_pose, draw_bbox_2d


def make_parser():
    parser = argparse.ArgumentParser("onnxruntime inference sample")
    parser.add_argument(
        "-m", "--model", type=str, default="yolox.onnx", help="Input your onnx model."
    )
    parser.add_argument(
        "--score-thr", type=float, default=0.3, help="Score threshold to filter the result."
    )
    parser.add_argument(
        "--input-shape", type=str, default="640,480", help="Specify an input shape for inference."
    )
    parser.add_argument(
        "--dataset", type=str, default='my', help="dataset type"
    )
    return parser


class YOLOXInferenceNode(Node):
    def __init__(self, session, input_shape, args):
        super().__init__("yolox_inference_node")

        # Subscribe to RealSense image topics
        self.sub_color = self.create_subscription(
            Image, '/camera/camera/color/image_raw', self.listener_callback_color, 10
        )
        self.sub_depth = self.create_subscription(
            Image, '/camera/camera/depth/image_rect_raw', self.listener_callback_depth, 10
        )

        # Create a publisher for object pose data
        self.pose_publisher = self.create_publisher(
            PoseStamped, '/object_pose', 10
        )

        self.cv_bridge = CvBridge()
        self.color_image = None
        self.depth_image = None
        self.session = session
        self.args = args
        self.input_shape = input_shape

        # Data storage for pose tracking
        self.translation_data = []
        self.rotation_data = []
        self.start_time = time.time()
        self.get_logger().info("YOLOX inference node ready!")

    def listener_callback_color(self, msg):
        """Callback function for processing color images."""
        self.color_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
        self.color_image = cv2.resize(self.color_image, (640, 480))
        self.process_frame()

    def listener_callback_depth(self, msg):
        """Callback function for processing depth images."""
        self.depth_image = self.cv_bridge.imgmsg_to_cv2(msg, "16UC1")
        self.depth_image = cv2.resize(self.depth_image, (640, 480))

    def filter_poses(self, tx, ty, tz, rvec):
        """Filter and store valid poses based on position consistency."""
        self.get_logger().info(f"Received pose data: tx={tx:.2f}, ty={ty:.2f}, tz={tz:.2f}")

        # Publish the pose data
        self.publish_pose(tx, ty, tz, rvec)

        # Store pose only if values are valid and within a reasonable range
        if tx is not None and ty is not None and tz is not None and rvec is not None:
            if len(self.translation_data) > 0:
                last_tx, last_ty, last_tz = self.translation_data[-1]
                if abs(tx - last_tx) < 0.5 and abs(ty - last_ty) < 0.5 and abs(tz - last_tz) < 0.5:
                    self.translation_data.append([tx, ty, tz])
                    self.rotation_data.append(rvec)
            else:
                self.translation_data.append([tx, ty, tz])
                self.rotation_data.append(rvec)

    def rotation_matrix_to_quaternion(self, R):
        """Convert a rotation matrix to a quaternion."""
        # Ensure the input is a valid rotation matrix
        if R is None or not isinstance(R, np.ndarray):
            return 0.0, 0.0, 0.0, 1.0   # Default quaternion (no rotation)

        try:
            trace = np.trace(R)
            if trace > 0:
                S = np.sqrt(trace + 1.0) * 2
                w = 0.25 * S
                x = (R[2, 1] - R[1, 2]) / S
                y = (R[0, 2] - R[2, 0]) / S
                z = (R[1, 0] - R[0, 1]) / S
            elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
                S = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
                w = (R[2, 1] - R[1, 2]) / S
                x = 0.25 * S
                y = (R[0, 1] + R[1, 0]) / S
                z = (R[0, 2] + R[2, 0]) / S
            elif R[1, 1] > R[2, 2]:
                S = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
                w = (R[0, 2] - R[2, 0]) / S
                x = (R[0, 1] + R[1, 0]) / S
                y = 0.25 * S
                z = (R[1, 2] + R[2, 1]) / S
            else:
                S = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
                w = (R[1, 0] - R[0, 1]) / S
                x = (R[0, 2] + R[2, 0]) / S
                y = (R[1, 2] + R[2, 1]) / S
                z = 0.25 * S

            return float(x), float(y), float(z), float(w)
        except Exception as e:
            self.get_logger().error(f"四元数转换错误: {str(e)}")
            return 0.0, 0.0, 0.0, 1.0  

    def publish_pose(self, tx, ty, tz, rotation_mat):
        """Publish pose data as a ROS2 PoseStamped message."""
        if tx is None or ty is None or tz is None or rotation_mat is None:
            return

        try:
            pose_msg = PoseStamped()
            pose_msg.header = Header()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = "camera_color_optical_frame"

            pose_msg.pose.position.x = float(tx)
            pose_msg.pose.position.y = float(ty)
            pose_msg.pose.position.z = float(tz)

            # Set orientation as a quaternio
            x, y, z, w = self.rotation_matrix_to_quaternion(rotation_mat)
            pose_msg.pose.orientation.x = float(x)
            pose_msg.pose.orientation.y = float(y)
            pose_msg.pose.orientation.z = float(z)
            pose_msg.pose.orientation.w = float(w)

            # Publish the pose message
            self.pose_publisher.publish(pose_msg)
            self.get_logger().info(f"Successfully published pose: tx={tx:.2f}, ty={ty:.2f}, tz={tz:.2f}")
        except Exception as e:
            self.get_logger().error(f"Error publishing pose data: {str(e)}")


    def compute_average(self):
        """计算过去10秒内的平均姿态"""
        if len(self.translation_data) > 0 and len(self.rotation_data) > 0:
            avg_translation = np.mean(self.translation_data, axis=0)
            avg_rotation = np.mean(self.rotation_data, axis=0)

            # 记录平均姿态
            self.get_logger().info(
                f"平均姿态 (tx: {avg_translation[0]:.2f}, ty: {avg_translation[1]:.2f}, tz: {avg_translation[2]:.2f})"
            )

            # 创建并发布平均姿态
            self.publish_pose(
                avg_translation[0],
                avg_translation[1],
                avg_translation[2],
                avg_rotation
            )

    def reset_pose_data(self):
        """重置姿态数据"""
        self.translation_data = []
        self.rotation_data = []
        self.start_time = time.time()

    def process_frame(self):
        """Process the color frame, run inference, and display results"""
        if self.color_image is None:
            return

      
        frame_resized = cv2.resize(self.color_image, self.input_shape, interpolation=cv2.INTER_LINEAR)
        img = frame_resized.transpose((2, 0, 1))
        img = np.ascontiguousarray(img, dtype=np.float32)

        
        ort_inputs = {self.session.get_inputs()[0].name: img[None, :, :, :]}
        output = self.session.run(None, ort_inputs)

       
        class_to_cuboid = get_cuboid_corner(dataset=self.args.dataset)
        camera_matrix = get_camera_matrix(dataset=self.args.dataset)
        class_names = get_class_names(dataset=self.args.dataset)
        dets = output[0]

       
        frame_2d_od = self.color_image.copy()
        if dets is not None:
            draw_bbox_2d(frame_2d_od, dets, class_names)

            # ensure draw_obj_pose return required params
            try:
                img, rotation_mat, tx, ty, tz = draw_obj_pose(
                    self.color_image, dets, class_names=class_names,
                    class_to_cuboid=class_to_cuboid, camera_matrix=camera_matrix
                )

                if rotation_mat is not None and tx is not None and ty is not None and tz is not None:
                    self.filter_poses(tx, ty, tz, rotation_mat)

                    
                    if time.time() - self.start_time > 10:
                        self.compute_average()
                        self.reset_pose_data()
            except Exception as e:
                self.get_logger().error(f"Error processing pose: {str(e)}")

        cv2.imshow('Detected Blocks', self.color_image)
        cv2.imshow('2D Bounding Boxes', frame_2d_od)

        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)

    # parase params
    args = make_parser().parse_args()
    input_shape = tuple(map(int, args.input_shape.split(',')))

    # load ONNX model
    model_path = "/home/mscrobotics2425laptop11/ros2_cam/src/onnx/your_model2.onnx"
    session = ort.InferenceSession(model_path)

    # initilize ROS2 node
    node = YOLOXInferenceNode(session, input_shape, args)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()