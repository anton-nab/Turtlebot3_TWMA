import rclpy
from geometry_msgs.msg import Twist

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import math 
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator



class Robot_goto(): 
    def __init__(self):
        self.node = rclpy.create_node('goto')
        self.initial_pose = None
        self.follow_me_finished = False

        self.subscription = self.node.create_subscription(
            PoseStamped, 
            '/amcl_pose',
            self.localization_callback,10)
        
        timer = self.node.create_timer(0.5, self.timer_callback)
        
        self.first_time = True
    
        self.goal_reached = False

    def timer_callback(self):
        # vérifier que timer callback fonctionne
        # Print le message dans self.localization callback 
        # condition de GOTO la pose sauvegardée
        # subscribe à un topic d'un  publiéefollow me puis dans la callback mettre booléen self.follow_me_finished
        self.node.get_logger().info('Timer callback')
        
        if self.follow_me_finished: 
            self.navigator.gotopose(self.pose)
    def localization_callback(self, msg):
        
        if self.first_time: 
        #sauvegarde de la pose 
            self.pose = msg.pose
            self.first_time = False
    
    
    def return_initial_pose(self):
        
        if self.initial_pose is not None: 
            goal_pose = PoseStamped()
            goal_pose.pose = self.initial_pose
            self.Navigation(goal_pose)

    def Navigation(self, goal_pose):   
        # Launch the ROS 2 Navigation Stack
        navigator = BasicNavigator()
        
        #set initial pose
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = navigator.get_clock()
        navigator.waitUntilNav2Active()
        
        path = navigator.getPath(initial_pose, goal_pose)
        smoothed_path = navigator.smoothPath(path)

        # Set the robot's goal poses
        goal_poses = [goal_pose]

    
        navigator.goThroughPoses(goal_poses)
            
        #Close the ROS2 Navigation Stack
        navigator.lifecycleShutdown()

        exit(0)


    def main(args=None):

        # Start the ROS 2 Python Client Library    
        rclpy.init(args=args)
        
        navigation_node =Robot_goto()
        
        # navigation_node.send_goal(goal_x=1.0, goal_y=1.0)
        
        # while not navigation_node.goal_reached and rclpy.ok():
        #     rclpy.spin_once(navigation_node.node)
        rclpy.spin(navigation_node)
        navigation_node.node.destroy_node()
        rclpy.shutdown()
        
 
if __name__ == '__main__':
    Robot_goto.main()