import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.qos import QoSProfile
from nav_msgs.msg import OccupancyGrid
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateThroughPoses, NavigateToPose
import numpy as np
from sensor_msgs.msg import Image
from std_srvs.srv import Empty
from std_msgs.msg import Bool

class DifferentialRobot(Node):
    def __init__(self):
        super().__init__('DifferentialRobot')

     #Create QoSProfile
        sanitized_map_qos = QoSProfile(depth=10, 
                                       durability=QoSDurabilityPolicy.TRANSIENT_LOCAL, 
                                       reliability=QoSReliabilityPolicy.RELIABLE)
        
        amcl_pose_qos = QoSProfile( depth=1,
                                    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                                    reliability=QoSReliabilityPolicy.RELIABLE,
                                    history=QoSHistoryPolicy.KEEP_LAST)

      #Create Publisher and Subscriber
        self.subscription = self.create_subscription( OccupancyGrid, '/global_costmap/costmap', self.OccupancyGrid_callback, 10 )   
        self.subscriber_localize = self.create_subscription(Bool, 'localization_complete', self.localization_callback, 10)         # subscribe to localize node to check if localization is complete or not
        self.nav_to_pose_client = ActionClient (self, NavigateToPose, 'navigate_to_pose')
        self.nav_through_poses_client = ActionClient (self, NavigateThroughPoses, 'navigate_through_poses')
        self.model_pose_sub = self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', self._amclPoseCallback, amcl_pose_qos) # Create the subscriber to 'amcl_pose' topic using the QoSProfile specified: amcl_pose_qos, with callback _amclPoseCallback
        self.energy_map_publisher = self.create_publisher(OccupancyGrid, '/sanitized_map', sanitized_map_qos) # Create the publisher to '/sanitized_map' topic using the QoSProfile specified:qos
    
        self.reinitialize_global_localization_client = self.create_client(Empty, '/reinitialize_global_localization')   # Create the client to the 'reinitialize_global_localization' service
        while not self.reinitialize_global_localization_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

      #Variables     
        self.localization_complete = False    # initialize the flag for localization to check if localization is complete or not
        self.map_data = None
        self.start_energy = False
        self.k=0
        self.occupancy_grid = None
        self.robot_pose = None
        self.initial_pose = Pose()
        self.goal_handle = None
        self.result_future = None
        self.feedback = None
        self.status = None
        self.first_iter = True
        self.cell_energy = None
        self.cell_sanitized = None
        self.cell_energy_level = None

#for sanitazer_task-4     
    def world_to_map(self, world_x, world_y):
        ''' 
        Convert world coordinates to map coordinates
        - Input: world_x, world_y
        - Output: map_x, map_y 
        '''

        origin = self.occupancy_grid.info.origin.position
        resolution = self.occupancy_grid.info.resolution

        map_x = (world_x - origin.x) / resolution
        map_y = (world_y - origin.y) / resolution

        return int(map_x), int(map_y)
    
    def map_to_world(self, map_x, map_y):
        '''
        Convert map coordinates to world coordinates
        - Input: map_x, map_y
        - Output: world_x, world_y
        '''
        origin = self.occupancy_grid.info.origin.position
        resolution = self.occupancy_grid.info.resolution

        world_x = map_x * resolution + origin.x
        world_y = map_y * resolution + origin.y

        return round(world_x, 2), round(world_y, 2)

    def OccupancyGrid_callback(self, msg): # called by Occupancy grid
        '''
        Callback function for the occupancy grid subscriber
        '''
        # self.get_logger().info('Occupancy grid received')
        self.occupancy_grid = msg 
        self.map_data = np.array(self.occupancy_grid.data)
        self.map_data = np.reshape(self.map_data, (self.occupancy_grid.info.height, self.occupancy_grid.info.width))

        self.map_data = np.where((self.map_data >= 0) & (self.map_data <= 97), 0, self.map_data)
        self.map_data = np.where(self.map_data > 97, 100, self.map_data)
        self.energy_map = self.Energy_map_update()
    
    def _amclPoseCallback(self, msg):
        '''
        Callback function for the AMCL pose subscriber
        '''
        self.initial_pose = msg.pose.pose
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        self.robot_pose = msg.pose.pose
        # print("Robot position: ", self.robot_x, self.robot_y)
    
    def goThroughPoses(self, poses):
        '''
        Sends a `NavigateThroughPoses` action request and waits for completion
        '''
       
        self.debug("Waiting for 'NavigateToPose' action server")
        while not self.nav_through_poses_client.wait_for_server(timeout_sec=1.0):
            self.info("'NavigateToPose' action server not available, waiting...")

        goal_msg = NavigateThroughPoses.Goal()
        goal_msg.poses = poses

        self.info('Navigating with ' + str(len(poses)) + ' goals.' + '...')
        send_goal_future = self.nav_through_poses_client.send_goal_async(goal_msg, self._feedbackCallback)
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            self.error('Goal with ' + str(len(poses)) + ' poses was rejected!')
            return False

        self.result_future = self.goal_handle.get_result_async()
        return True
    
    def goToPose(self, pose):
        '''
        Sends a `NavigateToPose` action request and waits for completion
        '''
        self.debug("Waiting for 'NavigateToPose' action server")
        while not self.nav_to_pose_client.wait_for_server(timeout_sec=1.0):
            self.info("'NavigateToPose' action server not available, waiting...")
            # self.energy_map = self.Energy_map_update()

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self.info('Navigating to goal: ' + str(pose.pose.position.x) + ' ' +
                        str(pose.pose.position.y) + '...')
        send_goal_future = self.nav_to_pose_client.send_goal_async(goal_msg, self._feedbackCallback)
        
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            self.error('Goal to ' + str(pose.pose.position.x) + ' ' +
                        str(pose.pose.position.y) + ' was rejected!')
            return False

        self.result_future = self.goal_handle.get_result_async()
        return True
    
    def _feedbackCallback(self, msg):
        '''
        Callback function for the feedback subscriber
        '''
        # local copy of the feedback callback for future use
        self.feedback = msg.feedback
        return

    def cancelNav(self):
        self.info('Canceling current goal.')
        if self.result_future:
            future = self.goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, future)
        return

    def isNavComplete(self):
        '''
        Checks if the navigation action is complete
        '''
        if not self.result_future:
            # task was cancelled or completed
            return True
        
        rclpy.spin_until_future_complete(self, self.result_future, timeout_sec=0.10)
        if self.result_future.result():
            self.status = self.result_future.result().status
            if self.status != GoalStatus.STATUS_SUCCEEDED:
                self.info('Goal with failed with status code: {0}'.format(self.status))
                return True
        else:
            # Timed out, still processing, not complete yet
            return False

        self.info('Goal succeeded!')
        return True

    def getFeedback(self):
        return self.feedback

    def getResult(self):
        return self.status

    def waitUntilNav2Active(self):
        self._waitForNodeToActivate('amcl')
        self._waitForInitialPose()
        self._waitForNodeToActivate('bt_navigator')
        self.info('Nav2 is ready for use!')
        return

    def _waitForNodeToActivate(self, node_name):
        self.debug('Waiting for ' + node_name + ' to become active..')
        node_service = node_name + '/get_state'
        state_client = self.create_client(GetState, node_service)
        while not state_client.wait_for_service(timeout_sec=1.0):
            self.info(node_service + ' service not available, waiting...')

        req = GetState.Request()
        state = 'unknown'
        while (state != 'active'):
            self.debug('Getting ' + node_name + ' state...')
            future = state_client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is not None:
                state = future.result().current_state.label
                self.debug('Result of get_state: %s' % state)
            time.sleep(2)
        return
    
    def info(self, msg):
        self.get_logger().info(msg)
        return

    def warn(self, msg):
        self.get_logger().warn(msg)
        return

    def error(self, msg):
        self.get_logger().error(msg)
        return

    def debug(self, msg):
        self.get_logger().debug(msg)
        return
    
    def current_energy(self, dx, dy):
        '''
        Calculate the energy for a cell based on the distance from the robot
        - Input: dx, dy (distance from the robot in x and y directions)
        - Output: energy for the cell
        '''
        distance_in_map_units = np.sqrt(dx**2 + dy**2)

        # Convert the distance to world units
        distance_in_world_units = distance_in_map_units * self.occupancy_grid.info.resolution
        pixel_2 = self.occupancy_grid.info.resolution**2
        pi = 100*1e-6
        process_speed_factor = 1e4 ### For simulation purposes only

        if distance_in_world_units > (0.1*self.occupancy_grid.info.resolution):
            energy = process_speed_factor*(pi * pixel_2)/(dx**2 + dy**2)
            # print("Energy: ", energy)
        else:
            energy = 0
        return energy

    def Energy_map_update(self):
        '''
        Update the energy grid based on the occupancy grid and the robot position
        '''
        
        if self.occupancy_grid is not None and self.start_energy == True:

            if self.k==0: #for index 0, create matrix with dimension of map.
                #create matrix of height x width.
                height= self.occupancy_grid.info.height  #height of occupancy grid
                width = self.occupancy_grid.info.width   #width of occupancy grid

                self.cell_energy = np.zeros((height, width), dtype=np.float32) # store energy of each cells
                self.cell_sanitized = np.zeros((height, width))
                self.cell_energy_level = np.zeros((height, width), dtype=np.uint8) 
                                
                self.k=1 # increase index 
            
            position = self.robot_pose

            if position is not None:
                robot_world_x, robot_world_y = position.position.x, position.position.y
                robot_x, robot_y = self.world_to_map(robot_world_x, robot_world_y)
                # energy_calculation = True
                num_directions = 360
                angle_increment = 360 / num_directions
                energy_threshold = 0.001 #1e-3 = 0.001

                for angle in range(0, 360, int(angle_increment)): #allows robot to spread energy 360 deg about it's self and sanitized
                    angle_rad = np.radians(angle) #convert to radians
                    ii, jj = 0, 0
                    
                    x_comp= robot_x
                    y_comp= robot_y
                    while(x_comp < 299 and y_comp < 264):   # Check if the cell is within the map boundaries                            
                        if self.map_data[y_comp, x_comp] == 100:  # Check if there is an obstacle in the current direction
                            self.cell_sanitized[y_comp, x_comp] = 1
                            break  # if an obstacle is exist in this direction stop spreading energy in this direction
                        else:
                            # Spreading energy and Calculate energy for the cell
                            calc_energy = self.current_energy(ii, jj) #how much energy the cell got this time
                            #print("cell Energies before added : ", self.cell_energy[y_comp, x_comp])
                            self.cell_energy[y_comp, x_comp] = self.cell_energy[y_comp, x_comp] + calc_energy #toltal energy cell obtained= prev. + current
                            #print("cell Energies: ", self.cell_energy[y_comp, x_comp])

                        #for Visualization and identify which cell is cell_sanitized
                        energy_value = self.cell_energy[y_comp, x_comp]

                        if energy_value >= (1e-8) and energy_value < (1e-6):
                            self.cell_energy_level[y_comp, x_comp] = 10 
                        elif energy_value >= (1e-7) and energy_value < (1e-5):
                            self.cell_energy_level[y_comp, x_comp] = 20 
                        elif energy_value >= (1e-5) and energy_value < (1e-3):
                            self.cell_energy_level[y_comp, x_comp] = 100  
                        elif energy_value >= (energy_threshold):
                                self.cell_energy_level[y_comp, x_comp] = -100
                                self.cell_sanitized[y_comp, x_comp] = 1  #is sanitized
                                # print("cell sanitized state: ",self.cell_sanitized[y_comp-1:y_comp+1, x_comp-1:x_comp+1])
                                # print("\n")


                        ii += np.cos(angle_rad)
                        jj += np.sin(angle_rad)
                        x_comp= robot_x + int(ii)
                        y_comp= robot_y + int(jj)
                        energy_value=0
                 
                # Publish the visualization on the sanitized_map topic
                sanitized_area = OccupancyGrid()
                sanitized_area.header.stamp = self.get_clock().now().to_msg()
                sanitized_area.header.frame_id = 'map'
                sanitized_area.info.width = self.occupancy_grid.info.width
                sanitized_area.info.height = self.occupancy_grid.info.height   
                sanitized_area.info.resolution = self.occupancy_grid.info.resolution
                sanitized_area.info.origin = self.occupancy_grid.info.origin
                sanitized_area.data = np.clip(self.cell_energy_level.flatten().astype(int), -128, 127).tolist()    #-128,     127
                self.energy_map_publisher.publish(sanitized_area)  #publish the sanitized area

            else:
                self.get_logger().info('Robot position not available yet')
                            
        else:
            # self.get_logger().info('Occupancy grid not received yet')
            pass
       

        return self.cell_energy
    
    def localization_callback(self, msg):
      # Callback function for the subscriber to the localization_complete topic to check if localization is complete or not
      self.localization_complete = msg.data

    