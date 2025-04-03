import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import time
from std_msgs.msg import Float32MultiArray  # Used for publishing the center and distance
from sensor_msgs.msg import CameraInfo

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header


class ImageDepthSubscriber(Node):
    def __init__(self, name):
        super().__init__(name)
        # Create subscriptions for color image, depth image, and camera info
        self.sub_color = self.create_subscription(
            Image, '/camera/camera/color/image_raw', self.listener_callback_color, 10
        )
        self.sub_depth = self.create_subscription(
            Image, '/camera/camera/depth/image_rect_raw', self.listener_callback_depth, 10
        )

        self.sub_camera_info = self.create_subscription(
            CameraInfo, '/camera/camera/color/camera_info', self.camera_info_callback, 10
        )
        self.pub = self.create_publisher(PoseStamped, '/bin_pose', 10)  # Publisher for pose

        # Camera intrinsic parameters
        self.fx, self.fy, self.cx, self.cy = None, None, None, None

        self.cv_bridge = CvBridge()
        self.color_image  = None
        self.depth_image = None
        self.start_time = time.time()
        self.pose_data = []

        # Define color ranges for detection
        self.color_ranges = {
        'red': ([0, 120, 70], [5, 255, 255]),  
        'yellow': ([20, 100, 100], [40, 255, 255]),
        'blue': ([90, 50, 50], [120, 255, 255]),
        'green': ([40, 40, 40], [80, 255, 255]),
        'purple': ([125, 50, 50], [150, 255, 255]),      
        'orange': ([5, 150, 150], [15, 255, 255])
        }
    

    def camera_info_callback(self, data):
        """Callback to extract camera intrinsic parameters (fx, fy, cx, cy)"""
        self.get_logger().info('Received camera info')
        K = data.k
        self.fx, self.fy = K[0], K[4]
        self.cx, self.cy = K[2], K[5]
        self.get_logger().info(f"fx: {self.fx}, fy: {self.fy}, cx: {self.cx}, cy: {self.cy}")

    def pixel_to_world(self, u, v, depth):
        """Convert pixel coordinates to world coordinates using depth and camera intrinsics"""
        Z = depth / 1000.0
        X = (u - self.cx) * Z / self.fx
        Y = (v - self.cy) * Z / self.fy
        return X, Y, Z
    

    def listener_callback_color(self, data):
        """Callback for receiving color image data"""
        self.get_logger().info('Receiving color video frame')
        self.color_image = self.cv_bridge.imgmsg_to_cv2(data, 'bgr8')

        self.detect_colored_cube(self.color_image)

    def listener_callback_depth(self, data):
        """Callback for receiving depth image data"""
        self.get_logger().info('Receiving depth video frame')
        self.depth_image = self.cv_bridge.imgmsg_to_cv2(data, '16UC1')
     
    def filter_poses(self, world_x, world_y, world_z,color):
        """Filter and store valid poses based on position consistency."""
        
        # Publish the pose data
        self.publish_pose_color(world_x, world_y, world_z,color)

        # Store pose only if values are valid and within a reasonable range
        if world_x is not None and world_y is not None and world_z is not None:
            if len(self.pose_data) > 0:
                last_x, last_y, last_z = self.pose_data[-1]
                if abs(world_x - last_x) < 0.5 and abs(world_y - last_y) < 0.5 and abs(world_z - last_z) < 0.5:
                    self.pose_data.append([world_x, world_y, world_z])
                   
            else:
                self.pose_data.append([world_x, world_y, world_z])
                
    def publish_pose_color(self,world_x,world_y,world_z,color):
        if world_x is None or world_y is None or world_z is None or color is None:
            return

        try:
         # Create PoseStamped message to publish
            pose_msg = PoseStamped()
            pose_msg.header = Header()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = "camera_bin_frame"

            pose_msg.pose.position.x = float(world_x)
            pose_msg.pose.position.y = float(world_y)
            pose_msg.pose.position.z = float(world_z)

            # Publish the pose message
            self.pub.publish(pose_msg)
            self.get_logger().info(f"Published pose for {color} at X: {world_x:.2f}, Y: {world_y:.2f}, Z: {world_z:.2f}")
        except Exception as e:
            self.get_logger().error(f"Error publishing pose data: {str(e)}")
            
    def compute_average(self,color):
        """计算过去10秒内的平均姿态"""
        if len(self.pose_data) > 0:
            avg_pose = np.mean(self.pose_data, axis=0)
         
            # record average position
            self.get_logger().info(
                f"average position (world_x: {avg_pose[0]:.2f}, world_y: {avg_pose[1]:.2f}, world_z: {avg_pose[2]:.2f})"
            )

            self.publish_pose_color(
                avg_pose[0],
                avg_pose[1],
                avg_pose[2],
                color
            )


    def reset_pose_data(self):
        """reset pose data"""
        self.pose_data = []
        self.start_time = time.time()

    def detect_colored_cube(self, image):
        """Detect colored cubes based on color range and depth data"""
        if self.depth_image is None:
            return

        hsv_img = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Loop through each defined color range for detection
        for color, (lower, upper) in self.color_ranges.items():
            scale_x = self.depth_image.shape[1] / self.color_image.shape[1]  # depth_width / rgb_width
            scale_y = self.depth_image.shape[0] / self.color_image.shape[0]  # depth_height / rgb_height

            # Create a mask for the current color range
            mask = cv2.inRange(hsv_img, np.array(lower), np.array(upper))
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for cnt in contours:
                if cv2.contourArea(cnt) > 600:
                    x, y, w, h = cv2.boundingRect(cnt)
                    
                    if w > 300 and h>300:
                        # Adjust the center coordinates according to depth scaling
                        center_x = int(x + w / 2)
                        center_y = int(y + h / 2)

                        if self.depth_image is not None:                        
                            center_y_depth = int(center_y * scale_y)
                            center_x_depth = int(center_x * scale_x)
                        
                        # Get the depth value at the center point of the bounding box
                        depth_value = self.depth_image[center_y_depth, center_x_depth].astype(float)
            
                        # Convert pixel coordinates to world coordinates
                        if center_x and center_y and depth_value and color is not None:
                            world_x, world_y, world_z = self.pixel_to_world(center_x, center_y, depth_value)
                            self.filter_poses(world_x,world_y,world_z,color)
                             # Draw a bounding box around the detected object
                            cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)
                            text = f"{color}"
                            text_xyz = f"Object detected at X: {world_x:.2f} m, Y: {world_y:.2f} m, Z: {world_z:.2f} "

                            # Add text with object color and world coordinates
                            offset_y = 20
                            cv2.putText(image, text, (x, y - offset_y), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                            cv2.putText(image, text_xyz, (x, y - 2 * offset_y), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                            if time.time() - self.start_time > 10:
                                self.compute_average(color)
                                self.reset_pose_data()

                       

                        # if world_x is not None and world_y is not None and world_z is not None:
                        #     self.filter_poses(world_x,world_y,world_z)

                       

     
        cv2.imshow('Cube Detector', image)
        cv2.waitKey(10)


def main(args=None):
    rclpy.init(args=args)
    node = ImageDepthSubscriber("image_depth_sub")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
