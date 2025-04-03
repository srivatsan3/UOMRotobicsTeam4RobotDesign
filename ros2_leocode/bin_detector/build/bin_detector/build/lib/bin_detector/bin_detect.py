import numpy as np 
import cv2 
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class RealsenseBinProcessor(Node):
    def __init__(self):
        super().__init__('realsense_bin_processor')
        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_callback,
            10
        ) 

        self.depth_sub = self.create_subscription(
            Image,
            '/camera/aligned_depth_to_color/image/raw',
            self.depth_callback,
            10
        )
        self.color_frame = None
        self.depth_frame = None

        # Define color ranges for detection
        self.color_ranges = {
        'red': ([0, 100, 100], [10, 255, 255]),
        'yellow': ([20, 100, 100], [40, 255, 255]),
        'blue': ([100, 100, 100], [130, 255, 255]),
        'green': ([40, 100, 100], [80, 255, 255]),
        'purple': ([130, 50, 50], [160, 255, 255]),
        'pink': ([140, 50, 50], [180, 255, 255]),
        'orange': ([10, 100, 100], [20, 255, 255])
        }
        self.get_logger().info('Start to detact the color of the bin. ')

def image_callback(self, msg):
    """ process RGB image """
    self.color_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    self.detect_color()

def depth_callback(self, msg):
    """ prcess depth image """
    self.depth_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
    self.detect_color()

def detect_color(self):

# Start a while loop 
 
    if self.color_frame is not None and self.depth_frame is not None:
    # 进行颜色识别处理
        hsv_frame = cv2.cvtColor(self.color_frame, cv2.COLOR_BGR2HSV)
      
        for color, (lower, upper) in self.color_ranges.items():
            # 创建颜色掩膜
            mask = cv2.inRange(hsv_frame, np.array(lower), np.array(upper))
        

            # 查找轮廓
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for contour in contours:
                if cv2.contourArea(contour) > 500: # 过滤小轮廓
                    x, y, w, h = cv2.boundingRect(contour)
                    # 绘制矩形框
                    cv2.rectangle(self.color_frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

                    # 获取该位置的深度值
                    depth_value = self.depth_frame[int(y + h / 2), int(x + w / 2)]
                    color_name = color
                    cv2.putText(self.color_frame,color_name,(x,y-10),
                                cv2.FONT_HERSHEY_SIMPLEX,0.9,(0,255,0),2)
                    # 处理深度信息，输出距离
                    self.get_logger().info(f"Distance at ({x + w / 2}, {y + h / 2}) is {depth_value} mm")

            # 显示图像
            cv2.imshow("Detected Colors and Shapes", self.color_frame)
            cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    bin_dectection_node = RealsenseBinProcessor()
    rclpy.spin(bin_dectection_node)
    bin_dectection_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

        # # Convert the imageFrame in 
        # # BGR(RGB color space) to 
        # # HSV(hue-saturation-value) 
        # # color space 
        # hsvFrame = cv2.cvtColor(imageFrame, cv2.COLOR_BGR2HSV) 

        # # Set range for red color and 
        # # define mask 
        # red_dark = np.array([136, 87, 111], np.uint8) 
        # red_light = np.array([180, 255, 255], np.uint8) 
        # red_mask = cv2.inRange(hsvFrame, red_dark, red_light) 

        # # Set range for green color and 
        # # define mask 
        # green_dark = np.array([25, 52, 72], np.uint8) 
        # green_light = np.array([102, 255, 255], np.uint8) 
        # green_mask = cv2.inRange(hsvFrame, green_dark, green_light) 

        # # Set range for blue color and 
        # # define mask 
        # blue_dark = np.array([94, 80, 2], np.uint8) 
        # blue_light = np.array([120, 255, 255], np.uint8) 
        # blue_mask = cv2.inRange(hsvFrame, blue_dark, blue_light) 
        
        # #Yellow Color
        # yellow_light=np.array([55,60,200],np.uint8)
        # yellow_dark=np.array([60,255,255],np.uint8)
        # yellow_mask = cv2.inRange(hsvFrame, yellow_dark, yellow_light) 

        # #Purple Color
        # purple_light = np.array([135, 50, 50], np.uint8)
        # purple_dark = np.array([150, 255, 255], np.uint8)
        # purple_mask = cv2.inRange(hsvFrame, purple_dark,purple_light)

        # #Pink Color
        # pink_light = np.array([165, 50, 50], np.uint8)
        # pink_dark = np.array([172, 255, 255], np.uint8)
        # pink_mask = cv2.inRange(hsvFrame, pink_dark,pink_light)

        # #Orange Color
        # orange_light = np.array([10, 100, 100], np.uint8)
        # orange_dark = np.array([25, 255, 255], np.uint8)
        # orange_mask = cv2.inRange(hsvFrame, orange_dark,orange_light)

        # # Morphological Transform, Dilation 
        # # for each color and bitwise_and operator 
        # # between imageFrame and mask determines 
        # # to detect only that particular color 
        # kernel = np.ones((5, 5), "uint8") 
        
        # # For red color 
        # red_mask = cv2.dilate(red_mask, kernel) 
        # res_red = cv2.bitwise_and(imageFrame, imageFrame, 
        #                         mask = red_mask) 
        
        # # For green color 
        # green_mask = cv2.dilate(green_mask, kernel) 
        # res_green = cv2.bitwise_and(imageFrame, imageFrame, 
        #                             mask = green_mask) 
        
        # # For blue color 
        # blue_mask = cv2.dilate(blue_mask, kernel) 
        # res_blue = cv2.bitwise_and(imageFrame, imageFrame, 
        #                         mask = blue_mask) 
        
        # # For yellow color 
        # yellow_mask = cv2.dilate(yellow_mask, kernel) 
        # res_yellow = cv2.bitwise_and(imageFrame, imageFrame, 
        #                         mask = yellow_mask) 
        
        # # For purple color 
        # purple_mask = cv2.dilate(purple_mask, kernel) 
        # res_purple = cv2.bitwise_and(imageFrame, imageFrame, 
        #                         mask = purple_mask) 
        
        # # For pink color 
        # pink_mask = cv2.dilate(pink_mask, kernel) 
        # res_pink = cv2.bitwise_and(imageFrame, imageFrame, 
        #                         mask = pink_mask) 
        
        # # For orange color 
        # orange_mask = cv2.dilate(orange_mask, kernel) 
        # res_orange = cv2.bitwise_and(imageFrame, imageFrame, 
        #                         mask = orange_mask) 

        # # Creating contour to track red color 
        # contours, hierarchy = cv2.findContours(red_mask, 
        #                                     cv2.RETR_TREE, 
        #                                     cv2.CHAIN_APPROX_SIMPLE) 
        
        # for pic, contour in enumerate(contours): 
        #     area = cv2.contourArea(contour) 
        #     if(area > 300): 
        #         x, y, w, h = cv2.boundingRect(contour) 
        #         imageFrame = cv2.rectangle(imageFrame, (x, y), 
        #                                 (x + w, y + h), 
        #                                 (0, 0, 255), 2) 
                
        #         cv2.putText(imageFrame, "Red Colour", (x, y), 
        #                     cv2.FONT_HERSHEY_SIMPLEX, 1.0, 
        #                     (0, 0, 255))     

        # # Creating contour to track green color 
        # contours, hierarchy = cv2.findContours(green_mask, 
        #                                     cv2.RETR_TREE, 
        #                                     cv2.CHAIN_APPROX_SIMPLE) 
        
        # for pic, contour in enumerate(contours): 
        #     area = cv2.contourArea(contour) 
        #     if(area > 300): 
        #         x, y, w, h = cv2.boundingRect(contour) 
        #         imageFrame = cv2.rectangle(imageFrame, (x, y), 
        #                                 (x + w, y + h), 
        #                                 (0, 255, 0), 2) 
                
        #         cv2.putText(imageFrame, "Green Colour", (x, y), 
        #                     cv2.FONT_HERSHEY_SIMPLEX, 
        #                     1.0, (0, 255, 0)) 

        # # Creating contour to track blue color 
        # contours, hierarchy = cv2.findContours(blue_mask, 
        #                                     cv2.RETR_TREE, 
        #                                     cv2.CHAIN_APPROX_SIMPLE) 
        # for pic, contour in enumerate(contours): 
        #     area = cv2.contourArea(contour) 
        #     if(area > 300): 
        #         x, y, w, h = cv2.boundingRect(contour) 
        #         imageFrame = cv2.rectangle(imageFrame, (x, y), 
        #                                 (x + w, y + h), 
        #                                 (255, 0, 0), 2) 
                
        #         cv2.putText(imageFrame, "Blue Colour", (x, y), 
        #                     cv2.FONT_HERSHEY_SIMPLEX, 
        #                     1.0, (255, 0, 0)) 
        
        # # Creating contour to track yellow color 
        # contours, hierarchy = cv2.findContours(yellow_mask, 
        #                                     cv2.RETR_TREE, 
        #                                     cv2.CHAIN_APPROX_SIMPLE) 
        # for pic, contour in enumerate(contours): 
        #     area = cv2.contourArea(contour) 
        #     if(area > 300): 
        #         x, y, w, h = cv2.boundingRect(contour) 
        #         imageFrame = cv2.rectangle(imageFrame, (x, y), 
        #                                 (x + w, y + h), 
        #                                 (0, 255, 255), 2) 
                
        #         cv2.putText(imageFrame, "Yellow Colour", (x, y), 
        #                     cv2.FONT_HERSHEY_SIMPLEX, 
        #                     1.0, (0, 255, 255)) 

        # # Creating contour to track purple color 
        # contours, hierarchy = cv2.findContours(purple_mask, 
        #                                     cv2.RETR_TREE, 
        #                                     cv2.CHAIN_APPROX_SIMPLE) 
        # for pic, contour in enumerate(contours): 
        #     area = cv2.contourArea(contour) 
        #     if(area > 300): 
        #         x, y, w, h = cv2.boundingRect(contour) 
        #         imageFrame = cv2.rectangle(imageFrame, (x, y), 
        #                                 (x + w, y + h), 
        #                                 (128, 0, 128), 2) 
                
        #         cv2.putText(imageFrame, "Purple Colour", (x, y), 
        #                     cv2.FONT_HERSHEY_SIMPLEX, 
        #                     1.0, (128, 0, 128)) 

        # # Creating contour to track pink color 
        # contours, hierarchy = cv2.findContours(pink_mask, 
        #                                     cv2.RETR_TREE, 
        #                                     cv2.CHAIN_APPROX_SIMPLE) 
        # for pic, contour in enumerate(contours): 
        #     area = cv2.contourArea(contour) 
        #     if(area > 300): 
        #         x, y, w, h = cv2.boundingRect(contour) 
        #         imageFrame = cv2.rectangle(imageFrame, (x, y), 
        #                                 (x + w, y + h), 
        #                                 (255, 192, 203), 2) 
                
        #         cv2.putText(imageFrame, "Pink Colour", (x, y), 
        #                     cv2.FONT_HERSHEY_SIMPLEX, 
        #                     1.0, (255, 192, 203)) 

        # # Creating contour to track orange color 
        # contours, hierarchy = cv2.findContours(orange_mask, 
        #                                     cv2.RETR_TREE, 
        #                                     cv2.CHAIN_APPROX_SIMPLE) 
        # for pic, contour in enumerate(contours): 
        #     area = cv2.contourArea(contour) 
        #     if(area > 300): 
        #         x, y, w, h = cv2.boundingRect(contour) 
        #         imageFrame = cv2.rectangle(imageFrame, (x, y), 
        #                                 (x + w, y + h), 
        #                                 (0, 165, 255), 2) 
                
        #         cv2.putText(imageFrame, "Orange Colour", (x, y), 
        #                     cv2.FONT_HERSHEY_SIMPLEX, 
        #                     1.0, (0, 165, 255))
        # # Program Termination 
        # cv2.imshow("Multiple Color Detection in Real-TIme", imageFrame) 
        # if cv2.waitKey(10) & 0xFF == ord('q'): 
        #     depthcam.release() 
        #     cv2.destroyAllWindows() 
        #     break