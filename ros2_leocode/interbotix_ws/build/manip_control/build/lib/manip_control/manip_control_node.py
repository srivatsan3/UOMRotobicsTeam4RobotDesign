from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
from interbotix_xs_modules.xs_robot.arm import InterbotixArmXSInterface
from interbotix_xs_modules.xs_robot.core import InterbotixRobotXSCore
import numpy as np
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from interbotix_common_modules.common_robot.exceptions import InterbotixException
from geometry_msgs.msg import Pose, PoseStamped
##Pose Limits
## X: 0.26-0.46      Try between 0.26-0.36
## Y: -0.20-0.2      Try between -0.1-0.1
## Z: 0.018-0.25     Try between 0.018-0.15


# X: 
# Y:
# Z:


class PoseSubscriber(Node):
   def __init__(self):
       super().__init__('pose_subscriber')
       self.obj = 0
       self.pose_subscription = self.create_subscription(
           msg_type=PoseStamped,
           topic='/object_pose',
           callback=self.grasp_callback,
           qos_profile=10)
       
       self.drop_subscription = self.create_subscription(
          msg_type=PoseStamped,
          topic='/bin_pose',
          callback= self.drop_callback,
          qos_profile=10
       )
      #  self.pose_subscription  # prevent unused variable warning

       self.wrist_to_gripper = 113.575
       self.shoulder_to_base = -93.1
       self.l1 = 106
       self.l2 = 100
       
       self.HTM = np.array([[ 0.57922797, -0.57922797, -0.57357644,  0.08727859],
         [ 0.70710678,  0.70710678,  0.        , -0.12093082],
         [ 0.40557979, -0.40557979,  0.81915204, -0.02735885],
         [ 0.        ,  0.        ,  0.        ,  1.        ]])
       self.avg_pose = []
       self.count = 0

       self.bot = InterbotixManipulatorXS("px100", "arm", "gripper")

   def inverse_kinematics_2dof(self,l1, l2, px, py, case, waist_ang):
    # Convert target point to standard convention (relative to X-axis)
      r = np.sqrt(px**2 + py**2)

      # Compute inverse kinematics
      cos_theta2 = (px**2 + py**2 - self.l1**2 - self.l2**2) / (2 * self.l1 * self.l2)
      if abs(cos_theta2) > 1:
         raise ValueError("Target is out of reach")

      theta2_1 = np.arccos(cos_theta2)
      theta2_2 = -theta2_1  # Second possible solution

      def compute_theta1(theta2):
         k1 = self.l1 + self.l2 * np.cos(theta2)
         k2 = self.l2 * np.sin(theta2)
         return np.arctan2(px, py) - np.arctan2(k2, k1)  # Adjust for Y-axis measurement

      theta1_1 = compute_theta1(theta2_1)
      theta1_2 = compute_theta1(theta2_2)

      solutions = [
         (np.degrees(theta1_1)-19.29, np.degrees(theta2_1)-90+19.29),
         (np.degrees(theta1_2)-19.29, np.degrees(theta2_2)-90+19.29)
      ]
      if case == 1:
         new_final_sols = [
            (waist_ang, solutions[0][0], solutions[0][1], - (solutions[0][0]+solutions[0][1])),
            (waist_ang,solutions[1][0], solutions[1][1], - (solutions[1][0]+solutions[1][1])),
            ]
      elif case == 2:
         new_final_sols = [
            (waist_ang, solutions[0][0], solutions[0][1], - (solutions[0][0]+solutions[0][1])+90),
            (waist_ang, solutions[1][0], solutions[1][1], - (solutions[1][0]+solutions[1][1])+90),
            ]

      return new_final_sols
   
   def manip_solutions(self,xg,yg,zg):
      gripper_point = [xg, yg, zg]
      solutions = dict()

      rg = np.sqrt(xg**2 + yg**2)
      waist = np.arcsin(yg/ rg) * 180/np.pi
      new_xg = rg

      try:
         sol1 = self.inverse_kinematics_2dof(self.l1, self.l2, new_xg-self.wrist_to_gripper+10, zg+self.shoulder_to_base+30, case = 1, waist_ang = waist)
      except:
         sol1 = None
         print('No Solution Exists, Target not reachable under Sol 1 criteria!')

      try:
         sol2 = self.inverse_kinematics_2dof(self.l1, self.l2, new_xg, zg+self.wrist_to_gripper+self.shoulder_to_base+30, case = 2, waist_ang = waist)
      except:
         sol2 = None
         print('No Solution Exists, Target not reachable under Sol 2 criteria!')

      solutions['SOL 1'] = sol1
      solutions['SOL 2'] = sol2

      return solutions

   def valid_solution_filter(self,solutions):
      sol1 = solutions['SOL 1']
      sol2 = solutions['SOL 2']

      limits = {'waist' : {'min':-100,'max':100},
            'shoulder':{'min':-111, 'max':107},
            'elbow':{'min':-121, 'max':92},
            'wrist':{'min':-100, 'max':123}}

      if sol1 != None:
         approved_sols = []
         for sol in sol1:
            if self.limit_checker(sol, limits) != False:
                  approved_sols.append(sol)

         solutions['SOL 1'] = approved_sols

      if sol2 != None:
         approved_sols = []
         for sol in sol2:
            if self.limit_checker(sol, limits) != False:
                  approved_sols.append(sol)
         
         solutions['SOL 2'] = approved_sols

      return solutions

   def limit_checker(self,sol, limits):
      if sol[0] > limits['waist']['min'] and sol[0] < limits['waist']['max']:
            if sol[1] > limits['shoulder']['min'] and sol[0] < limits['shoulder']['max']:
                  if sol[2] > limits['elbow']['min'] and sol[1] < limits['elbow']['max']:
                     if sol[3] > limits['wrist']['min'] and sol[2] < limits['wrist']['max']:
                        return sol
      return False

#    def turn_on_request(self):  # Method for Task 1 to turn on the robot
#        while not self.publishers.wait_for_publisher(
#                timeout_sec=1.0):  # Loop every 1 second to wait for the server to become available
#            self.get_logger().info(f'service not available, waiting...')
   def inverse_kinematics_computation(self, xg,yg,zg):
      return self.valid_solution_filter(self.manip_solutions(xg,yg,zg))
   
   def solution_chooser(self, solutions, case, strict = False):
      if case == 1:
         if solutions['SOL 1'] != None and len(solutions['SOL 1']) > 0:
            ja = solutions['SOL 1'][0]
            self.flag = 1
         elif strict == False:
            if solutions['SOL 2'] != None and len(solutions['SOL 2']) > 0:
               ja = solutions['SOL 2'][0]
               self.flag = 2
            else:
               print('No Valid Solutions Found')
               ja = None
         elif strict == True:
            print('No valid CASE 1 Solution Found')
           
      elif case == 2:
         if solutions['SOL 2'] != None and len(solutions['SOL 2']) > 0:
            ja = solutions['SOL 2'][0]
            self.flag = 2
         elif strict == False:
            if solutions['SOL 1'] != None and len(solutions['SOL 1']) > 0:
               ja = solutions['SOL 1'][0]
               self.flag = 1
            else:
               print('No Valid Solutions Found')
               ja = None
         elif strict == True:
            print('No valid CASE 2 Solution Found')
      
      return ja, self.flag


   def grasp_callback(self, msg):
      bot = self.bot

      if self.obj == 0:

         self.count = self.count + 1

         self.get_logger().info(
               f"Received pose message - Position: ({msg.pose.position.x}, {msg.pose.position.y}, {msg.pose.position.z}), "
               f"Orientation: ({msg.pose.orientation.w}, {msg.pose.orientation.x}, {msg.pose.orientation.y}, {msg.pose.orientation.z})")
         self.x_pos = msg.pose.position.x
         self.y_pos = msg.pose.position.y
         self.z_pos = msg.pose.position.z

         self.avg_pose.append([self.x_pos, self.y_pos, self.z_pos])

      if self.count == 2:
         self.avg_pose = np.array(self.avg_pose)
         self.mean_pose = np.median(self.avg_pose, axis = 0)

         self.x_pos = self.mean_pose[0]
         self.y_pos = self.mean_pose[1]
         self.z_pos = self.mean_pose[2]

         print('MEAN POSE:', self.mean_pose)
         

      if self.obj == 0 and self.count == 2:
         # self.get_logger().info(
         #    f"Received pose message - Position: ({msg.pose.position.x}, {msg.pose.position.y}, {msg.pose.position.z}), "
         #    f"Orientation: ({msg.pose.orientation.w}, {msg.pose.orientation.x}, {msg.pose.orientation.y}, {msg.pose.orientation.z})")
         # self.x_pos = msg.pose.position.x
         # self.y_pos = msg.pose.position.y
         # self.z_pos = msg.pose.position.z

         self.dcm_cords = np.array([self.x_pos/1000, self.y_pos/1000, self.z_pos/1000, 1]) #+ np.array([-25,15,0,0])/1000
         self.manip_cords = (np.linalg.inv(self.HTM) @ self.dcm_cords)

         print(self.manip_cords)


         self.x_pos = 1000*self.manip_cords[0] - 10
         self.y_pos = 1000*self.manip_cords[1] + 30
         self.z_pos = 1000*self.manip_cords[2] + 20
         self.case = 1

         # if self.x_pos > 210:
         self.x_pos = self.x_pos + (self.x_pos-200)*1.01

         if self.y_pos > 10:
            self.y_pos = self.y_pos + (self.y_pos - 20)*1.01
         # if self.case == 2:
         #    self.x_pos = self.x_pos + 10

         # if self.y_pos > 0:
         #    self.x_pos = self.x_pos * 0.95
         # else:
         #    self.x_pos = self.x_pos * 1.1
            

         final_solutions = self.inverse_kinematics_computation(self.x_pos, self.y_pos, self.z_pos)
         print('ALL AVAILABLE SOLUTIONS:')
         print(final_solutions)
         ja, self.flag = self.solution_chooser(final_solutions, case=self.case)
         if ja != None:
            print('Final Solution is of type :', self.flag)

            print('Joint Angles are: ')
            print('Waist   :', ja[0],'\n','Shoulder:', ja[1],'\n','Elbow:', ja[2],'\n','Wrist:', ja[3] )

            wa,sh,el,wr = np.radians(ja)

            if self.flag == 2:
               wr = wr - np.radians(5)

            print('Releasing Gripper')
            bot.gripper.release()
            print('Gripper Released, Going to Home Pose')
            bot.arm.go_to_home_pose(moving_time=5)
            
            print('Going to safe default Pose : ', f'{self.x_pos/1000}, {self.y_pos/1000}, {0.3}')
            bot.arm.set_ee_pose_components(x = self.x_pos/1000, y = self.y_pos/1000, z = 0.1)
            print('Going to final Pose')
            if self.flag == 1:
               bot.arm.set_joint_positions([wa,sh,el,wr], moving_time=2)
            if self.flag == 2:
               bot.arm.set_joint_positions([wa,sh,el,wr], moving_time=4)

            bot.gripper.grasp()
            print('GRIPPER INFOR',bot.gripper.get_finger_position(), bot.gripper.get_gripper_position())
            time.sleep(2)
            print('Gripper got the Object!')
            self.obj = 1
            bot.arm.go_to_sleep_pose(moving_time=5)
            # bot.gripper.release()

            self.avg_pose = []
            self.count = 0

   def drop_callback(self, msg):
      print('DROPPER INITIATED!!')
      bot = self.bot
      if self.obj == 1:
         
         self.get_logger().info(
            f"Received pose message - Position: ({msg.pose.position.x}, {msg.pose.position.y}, {msg.pose.position.z}), "
            f"Orientation: ({msg.pose.orientation.w}, {msg.pose.orientation.x}, {msg.pose.orientation.y}, {msg.pose.orientation.z})")
         self.x_pos = msg.pose.position.z
         self.z_pos = msg.pose.position.y 
         self.y_pos = msg.pose.position.x * 1.1
         # self.z_pos = msg.pose.position.z
         # self.z_pos = 0.1

         if self.y_pos > 0:
            self.x_pos = self.x_pos * 0.95
         else:
            self.x_pos = self.x_pos * 1.1

         self.dcm_cords = np.array([self.x_pos/1000, self.y_pos/1000, self.z_pos/1000, 1]) #+ np.array([-25,15,0,0])/1000
         self.manip_cords = (np.linalg.inv(self.HTM) @ self.dcm_cords)

         print(self.manip_cords)

         self.x_pos = 1000*self.manip_cords[0] + 110
         if self.x_pos > 370:
            self.x_pos = self.x_pos - 60
         self.y_pos = 1000*self.manip_cords[1]
         # self.z_pos = 1000*self.manip_cords[2]
         self.z_pos = 0.1
         self.case = 1
         final_solutions = self.inverse_kinematics_computation(self.x_pos, self.y_pos, self.z_pos)
         print('ALL AVAILABLE SOLUTIONS:')
         print(final_solutions)
         ja, self.flag = self.solution_chooser(final_solutions, case=self.case)

         if ja!= None:
            print('Final Solution is of type :', self.flag)
            print('Joint Angles are: ')
            print('Waist   :', ja[0],'\n','Shoulder:', ja[1],'\n','Elbow:', ja[2],'\n','Wrist:', ja[3] )

            wa,sh,el,wr = np.radians(ja)
            print('Going to safe default Pose : ', f'{self.x_pos/1000}, {self.y_pos/1000}, {0.3}')
            bot.arm.set_ee_pose_components(x = self.x_pos/1000, y = self.y_pos/1000, z = 0.1)
            print('Going to final Pose')
            
            if self.flag == 1:
               bot.arm.set_joint_positions([wa,sh,el,wr], moving_time=2)
            if self.flag == 2:
               bot.arm.set_joint_positions([wa,sh,el,wr], moving_time=4)
            bot.gripper.release()
            bot.arm.go_to_sleep_pose(moving_time=5)

            self.obj = 0

def main(args=None):
   rclpy.init(args=args)
   pose_subscriber = PoseSubscriber()
   rclpy.spin(pose_subscriber)
   rclpy.shutdown()

if __name__=='__main__':
   main()