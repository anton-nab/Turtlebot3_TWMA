import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator

class Robot_goto(): 

    def __init__(self):
        self.initial_pose = None
        self.follow_me_finished = False
        self.follow_me_Debut = False
        self.goal_reached = False
        self.nav = BasicNavigator()

        # Créer le subscriber pour amcl_pose
        self.node = rclpy.create_node('robot_goto')
        self.robot_sub = self.node.create_subscription(
            PoseStamped, 
            '/amcl_pose',
            self.localization_callback,
            10)

        # Créer le timer
        timer = self.node.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        if self.follow_me_finished: 
            self.node.get_logger().info(f"pose ={self.initial_pose}")
            self.Navigation(self.initial_pose)

    def localization_callback(self, msg):
        self.node.get_logger().info('Localization callback triggered.')
        if not self.follow_me_Debut: 
            self.initial_pose = msg.pose
            self.node.get_logger().info(f"Initial pose saved: {self.initial_pose}")
            self.follow_me_Debut = True

    def Navigation(self, goal_pose): 
        self.nav.waitUntilNav2Active()
        self.nav.goToPose(goal_pose)
        self.nav.lifecycleShutdown()

def main(args=None):
    rclpy.init(args=args)
    robot = Robot_goto()
    rclpy.spin(robot.node)
    robot.node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
