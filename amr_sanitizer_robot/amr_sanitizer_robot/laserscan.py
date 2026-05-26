
'''
ROS2 Humble node to control a turtlebot3 using the velocity topic

'''

import rclpy
from rclpy.node import Node
#from amr_sanitizer_robot.robot_class import DifferentialRobot

#for laserscan
from geometry_msgs.msg import Twist  # Import \cmd_vel topic message type
from sensor_msgs.msg import LaserScan  # Import Laser scanner message type
from std_msgs.msg import Bool


# FSM State definition
TB3_MOVING = 0
TB3_ROTATING = 1
TB3_RECOVERY = 2

THRESHOLD_SAFE_DISTANCE = 1.0 # units in meters


class rdv(Node):
    def __init__(self):
        super().__init__('rdv')

      #Publisher
        self.VelocityController = self.create_publisher(Twist, '/cmd_vel', 10)
      #and Subscriber  
        self.LaserScanner = self.create_subscription(LaserScan, '/scan', self.laser_callback, 10)
        self.BoolSubscriber = self.create_subscription(Bool, 'localization_complete', self.localization_callback, 10)
      
      #Create Rate to specify the frequency of the main control loop
        self.rate = self.create_rate(30) #Hz

      #initialize the states
        self.tb3_state = TB3_MOVING
        self.laser_data = None
        self.localization_complete = False

  #functions
    def move(self):
        # Function to move forward the turtlebot
        vel_msg = Twist()
        vel_msg.linear.x = 0.15
        vel_msg.angular.z = 0.05

        self.VelocityController.publish(vel_msg)
        # self.node.get_logger().info(f'Publishing new velocity X: {vel_msg.linear.x} Z: {vel_msg.angular.z}')
        return

    def rotate(self):
        # Function to rotate the turtlebot
        vel_msg = Twist()
        vel_msg.linear.x = 0.0
        vel_msg.angular.z = 0.1

        self.VelocityController.publish(vel_msg)
        # self.node.get_logger().info(f'Publishing new velocity X: {vel_msg.linear.x} Z: {vel_msg.angular.z}')

        return

    def stop(self):
        # Function to stop the turtlebot
        vel_msg = Twist()
        vel_msg.linear.x = 0.0
        vel_msg.angular.z = 0.0

        self.VelocityController.publish(vel_msg)
        # self.node.get_logger().info(f'Publishing new velocity X: {vel_msg.linear.x} Z: {vel_msg.angular.z}')
        return

    def laser_callback(self, msg):
        # Function to store the laser scanner data
        self.laser_data = msg
        print("laser data received")
        return self.laser_data

    def check_collision(self):
        # check for collision using laser scanner 
        if self.laser_data is None:
            print("No laser data received")
            return False
     
        laser_data = self.laser_data.ranges[-25:] + self.laser_data.ranges[:25]  #in the range of 25deg to the left and right

        for i in range(len(laser_data)):
            if laser_data[i] < THRESHOLD_SAFE_DISTANCE:
                return True
            
        # self.tb3_state = TB3_MOVING
        return False

    def shutdown_node(self):
        # Function to shut down the node
        self.stop()
        self.destroy_node()  #self.node.destroy_node()
        rclpy.shutdown()

    def control_loop(self):
        # Main control loop to control the movement of the turtlebot based on the laser scanner data
        while rclpy.ok():

            rclpy.spin_once(self)
            # TB3 moving forward
            if self.tb3_state == TB3_MOVING:
                self.move()
                # Control with laser scanner
                if self.check_collision():
                   self.tb3_state = TB3_ROTATING
            # TB3 is rotating
            elif self.tb3_state == TB3_ROTATING:
                self.rotate()
                if not self.check_collision():
                   self.tb3_state = TB3_MOVING
            # TB3 is stopped and is not moving
            elif self.tb3_state == TB3_RECOVERY:
                # Example of additional usfull state
                # !!!!
                continue

    def localization_callback(self, msg):
        # Function to check if localization has been completed to shut down the node
        self.localization_complete = msg.data
        if self.localization_complete:
            print("Localization complete, so Lets stop the laserscan node.")
            self.shutdown_node()


def main(args=None):
   # main function to run the node
   rclpy.init()
   robot = rdv()
   robot.control_loop()
   rclpy.spin(robot)    
   robot.destroy_node()
   rclpy.shutdown()

if __name__ == '__main__':
   main()
