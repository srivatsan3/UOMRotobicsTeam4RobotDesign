import rclpy
from rclpy.node import Node
# import onnxruntime as ort
# import numpy as np
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import argparse
import os
import cv2
import numpy as np
import onnxruntime as ort
import time
from  yolox_pose_estimator.object_pose_utils_onnx import get_cuboid_corner, get_camera_matrix, get_class_names, draw_obj_pose, draw_bbox_2d


def make_parser():
    parser = argparse.ArgumentParser("onnxruntime inference sample")
    parser.add_argument(
        "-m",
        "--model",
        type=str,
        default="yolox.onnx",
        help="Input your onnx model.",
    )
    parser.add_argument(
        "--score-thr",
        type=float,
        default=0.3,
        help="Score threshold to filter the result.",
    )
    parser.add_argument(
        "--input-shape",
        type=str,
        default="640,480",   
        help="Specify an input shape for inference.",
    )
    parser.add_argument(
        "--dataset",
        type=str,
        default='my',   #####   lmo
        help="dataset type",
    )
    return parser

class YOLOXInferenceNode(Node):
    def __init__(self, session, input_shape, args):
        super().__init__("yolox_inference_node")
        

       
 

        # 订阅Realsense图像话题
        self.sub_color = self.create_subscription(
            Image, '/camera/camera/color/image_raw', self.listener_callback_color, 10
        )
        self.sub_depth = self.create_subscription(
            Image, '/camera/camera/depth/image_rect_raw', self.listener_callback_depth, 10
        )
        self.cv_bridge = CvBridge()
        self.color_image  = None
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
        """Callback function for color image"""
        self.color_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
        self.color_image = cv2.resize(self.color_image, (640, 480))
        self.process_frame()

    def listener_callback_depth(self, msg):
        """Callback function for depth image"""
       
        self.depth_image = self.cv_bridge.imgmsg_to_cv2(msg, "16UC1")    
        self.depth_image = cv2.resize(self.depth_image, (640, 480))

    def filter_poses(self, tx, ty, tz, rvec):
        """Filter and store valid poses."""
    # Only store poses if they are not None and fit within reasonable thresholds
        if tx is not None and ty is not None and tz is not None and rvec is not None:
            if len(self.translation_data) > 0:
                last_tx, last_ty, last_tz = self.translation_data[-1]
                if abs(tx - last_tx) < 0.5 and abs(ty - last_ty) < 0.5 and abs(tz - last_tz) < 0.5:
                    self.translation_data.append([tx, ty, tz])
                    self.rotation_data.append(rvec)
            else:
                self.translation_data.append([tx, ty, tz])
                self.rotation_data.append(rvec)

    def compute_average(self):
        """Compute average pose over the last 10 seconds."""
        if len(self.translation_data) > 0 and len(self.rotation_data) > 0:
            avg_translation = np.mean(self.translation_data, axis=0)
            avg_rotation = np.mean(self.rotation_data, axis=0)

            # Log the average pose
            self.get_logger().info(f"Avg Pose (tx: {avg_translation[0]:.2f}, ty: {avg_translation[1]:.2f}, tz: {avg_translation[2]:.2f})")
            self.get_logger().info(f"Avg Rotation: {avg_rotation}")

    def reset_pose_data(self):
        """Reset the pose data after every 10 seconds."""
        self.translation_data = []
        self.rotation_data = []
        self.start_time = time.time()

    
    def process_frame(self):
        """Process the color frame, run inference, and display results."""
        if self.color_image is None:
            return
        
        # Resize and prepare the image
        frame_resized = cv2.resize(self.color_image, self.input_shape, interpolation=cv2.INTER_LINEAR)
        img = frame_resized.transpose((2, 0, 1))
        img = np.ascontiguousarray(img, dtype=np.float32)

        # Inference with ONNX model
        ort_inputs = {self.session.get_inputs()[0].name: img[None, :, :, :]}
        output = self.session.run(None, ort_inputs)

        # Get dataset-specific cuboid, camera matrix, and class names
        class_to_cuboid = get_cuboid_corner(dataset=self.args.dataset)
        camera_matrix = get_camera_matrix(dataset=self.args.dataset)
        class_names = get_class_names(dataset=self.args.dataset)
        dets = output[0]
        # self.get_logger.info(f" dets:{dets}")
        # Draw detection results
        frame_2d_od = self.color_image.copy()
        if dets is not None:
            # cls = class_names(dets[0][0])
           
            draw_bbox_2d(frame_2d_od, dets, class_names)
            img, rotation_mat, tx,ty,tz  = draw_obj_pose(self.color_image, dets, class_names=class_names, class_to_cuboid=class_to_cuboid, camera_matrix=camera_matrix)
            self.filter_poses(tx,ty,tz,rotation_mat)
              # Calculate the average pose over time if 10 seconds have passed

            # self.get_logger().info(f"time duration: {time.time() - self.start_time}")
            if time.time() - self.start_time > 10:
                self.compute_average()  # Compute and log the average pose
                self.reset_pose_data()   # Reset data for the next 10-second window

    
            
    
        # Display results in OpenCV windows
        cv2.imshow('Detected Blocks', self.color_image)
        cv2.imshow('2D Bounding Boxes', frame_2d_od)

        # Wait for 'q' key to exit
        if cv2.waitKey(1) & 0xFF == ord('q'):
            rclpy.shutdown()    


def main(args=None):
    rclpy.init(args=args)

    # 解析参数
    args = make_parser().parse_args()
    input_shape = tuple(map(int, args.input_shape.split(',')))

    # 加载 ONNX 模型
    model_path = "/home/mscrobotics2425laptop5/Downloads/onnx/your_model2.onnx"
    session = ort.InferenceSession(model_path)
    # 初始化 ROS2 节点
    node = YOLOXInferenceNode(session, input_shape, args)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    # 清理资源
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()

# #
# def main(args=None):
#     rclpy.init(args=args)
#     node = YOLOXInferenceNode()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     args = make_parser().parse_args()
#     input_shape = tuple(map(int, args.input_shape.split(',')))

#     # Load the YOLOX ONNX model
#     session = ort.InferenceSession(args.model)

#     # Initialize ROS2 Node and Subscriber
#     rclpy.init()
#     node = YOLOXInferenceNode('realtime_object_detection', session, input_shape, args)
    
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass

#     # Cleanup
#     node.destroy_node()
#     rclpy.shutdown()
#     cv2.destroyAllWindows()

#     def image_callback(self, msg):
#         # 将ROS2图像消息转换为OpenCV格式
#         cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
#         input_tensor = self.preprocess(cv_image)

#         # 推理
#         outputs = self.session.run(None, {"images": input_tensor})

#         # 处理输出
#         boxes, scores, class_ids = self.postprocess(outputs)
#         self.draw_results(cv_image, boxes, scores, class_ids)

#     def preprocess(self, img):
#         img = cv2.resize(img, (640, 480))
#         img = img.transpose(2, 0, 1).astype(np.float32) / 255.0
#         return np.expand_dims(img, axis=0)

#     def postprocess(self, outputs):
#         boxes = outputs[0][:, :4]
#         scores = outputs[0][:, 4]
#         class_ids = outputs[0][:, 5]
#         return boxes, scores, class_ids

#     def draw_results(self, img, boxes, scores, class_ids):
#         for (box, score, class_id) in zip(boxes, scores, class_ids):
#             if score > 0.5:
#                 x1, y1, x2, y2 = map(int, box)
#                 cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
#                 cv2.putText(img, f"ID: {int(class_id)}, {score:.2f}", (x1, y1 - 10),
#                             cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
#         cv2.imshow("Detection", img)
#         cv2.waitKey(1)



# if __name__ == "__main__":
#     main()
