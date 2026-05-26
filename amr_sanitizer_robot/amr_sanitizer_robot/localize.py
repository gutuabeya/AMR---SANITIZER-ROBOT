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
from std_srvs.srv import Empty
from std_msgs.msg import Header
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
#from amr_sanitizer_robot.robot_class import DifferentialRobot

class DifferentialRobot(Node):
    def __init__(self):
        super().__init__('DifferentialRobot')

      #Publisher and Subscriber
        self.publisher_localize = self.create_publisher(Bool, 'localization_complete', 10)
        #self.BoolSubscriber = self.create_subscription(Bool, 'localization_complete', self.bool_callback, 10)
       # Create the publisher to 'initialpose' for the initialization
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', 10)

        qos = QoSProfile(depth=10, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL, reliability=QoSReliabilityPolicy.RELIABLE)
        amcl_pose_qos = QoSProfile(
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1)

        # Create the client to the 'reinitialize_global_localization' service
        self.reinitialize_global_localization_client = self.create_client(Empty, '/reinitialize_global_localization')
        while not self.reinitialize_global_localization_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        # Create the subscriber to 'amcl_pose' topic using the QoSProfile specified
        self.model_pose_sub = self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', self._amclPoseCallback_loc, amcl_pose_qos)
        # Create a subscriber to the 'odom' topic
        self.odometry_sub = self.create_subscription(Odometry,'odom',self.odometry_callback, 10)

        self.nav_to_pose_client = ActionClient (self, NavigateToPose, 'navigate_to_pose')
        self.nav_through_poses_client = ActionClient (self, NavigateThroughPoses, 'navigate_through_poses')
        #self.model_pose_sub = self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', self._amclPoseCallback, amcl_pose_qos)

      #Variables        
        self.occupancy_grid = None
        self.robot_pose = None
        self.initial_pose = Pose()
        self.goal_handle = None
        self.result_future = None
        self.feedback = None
        self.status = None
        self.first_iter = True

      #for localization
        self.initial_pose_received = False
        self.average_covariance = float('inf')
        self.x_cov = float('inf')
        self.y_cov = float('inf')
        self.z_cov = float('inf')
        self.localization_done = False   # initialize the flag for localization to check if localization is complete or not

    def odometry_callback(self, msg):
        # Update the robot's initial position using odometry data
        self.initial_pose = msg.pose.pose 

    def _amclPoseCallback_loc(self, msg):
        # Update the robot's initial position using amcl_pose data and covariance
            self.initial_pose = msg.pose.pose
            self.x_cov = msg.pose.covariance[0]
            self.y_cov = msg.pose.covariance[7]
            self.z_cov = msg.pose.covariance[35]
            self.average_covariance = (self.x_cov + self.y_cov) / 2
            self.robot_pose = msg.pose.pose.position.x, msg.pose.pose.position.y
            # print('covariance:',msg.pose.covariance)

def main(args=None):
    rclpy.init(args=args)

    robot = DifferentialRobot()
    # Wait until the initial pose is received
    while robot.initial_pose is None:
        rclpy.spin_once(robot)

    # Set the initial pose of the robot, this is the first guess of the robot's position
    inital_pose = PoseWithCovarianceStamped()  
    inital_pose.header = Header()
    inital_pose.header.frame_id = 'map'
    inital_pose.pose.pose.position.x = robot.initial_pose.position.x
    inital_pose.pose.pose.position.y = robot.initial_pose.position.y
    inital_pose.pose.pose.position.z = 0.0

    robot.initial_pose_pub.publish(inital_pose)
    print("Robot position initial guess: ", robot.initial_pose)
 
    request = Empty.Request()
    future = robot.reinitialize_global_localization_client.call_async(request)
    min_covariance_x = 1.0
    min_covariance_y = 1.5
    min_covariance_z = 0.5
    covariance_threshold = (min_covariance_x + min_covariance_y) / 2

    iteration = 0
    # Keep setting the initial pose until the average covariance is below the threshold
    while robot.x_cov > min_covariance_x or robot.y_cov > min_covariance_y or robot.z_cov > min_covariance_z:
    
        # print('x_cov: ', robot.x_cov)
        # print('y_cov: ', robot.y_cov)
        # print('z_cov: ', robot.z_cov)
    
        rclpy.spin_once(robot)
        iteration += 1

        print('iteration: ', iteration)
        
        if iteration % 20000 == 0 and robot.average_covariance > 1.25 * covariance_threshold:  
               
            print('Robot exact location is not know. Reinitializing localization...')
            rclpy.spin_once(robot)
            request = Empty.Request()
            future = robot.reinitialize_global_localization_client.call_async(request)
            inital_pose = PoseWithCovarianceStamped()  
            inital_pose.header = Header()
            inital_pose.header.frame_id = 'map'
            inital_pose.pose.pose.position.x = robot.initial_pose.position.x
            inital_pose.pose.pose.position.y = robot.initial_pose.position.y
            inital_pose.pose.pose.position.z = 0.0
       
    robot.localization_done = True
    print('Localization complete!')
    if robot.localization_done:
        print('Localization complete!')
        robot.publisher_localize.publish(Bool(data=True))
 
    rclpy.spin(robot)
    robot.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 

