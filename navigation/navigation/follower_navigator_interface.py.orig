from geometry_msgs.msg import PoseStamped
from navigation.robot_navigator_interface import BasicNavigator, TaskResult
import rclpy
from rclpy.parameter import Parameter
from rclpy.node import Node
import tf_transformations


class FollowerNavigationDemoInterface(Node):
    """
    Example of a class that uses the BasicNavigator class to navigate the robot.
    """

    def __init__(self, node_name="follower_navigation", namespace="follower"):
        super().__init__(node_name)
        
        # Declare the follower parameter
        # This parameter is used to determine the task of the follower robot
        self.declare_parameter("follower", "init")
        # get the parameter value
        self._task_param = (
            self.get_parameter("follower").get_parameter_value().string_value
        )

        # Since we are using Gazebo, we need to set the use_sim_time parameter to True
        self._sim_time = Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, True)
        self.set_parameters([self._sim_time])

        # Navigator
        self._follower_navigator = BasicNavigator(node_name, namespace)
        # Initial pose
        self._follower_initial_pose = PoseStamped()

        # Task the follower robot should perform
        if self._task_param == "init":
            self.localize()
        elif self._task_param == "goal":
            self.localize()
            self.navigate(-9.752330, -0.288182)
        elif self._task_param == "waypoints":
            self.localize()
            self.follow_waypoints()

        # subscriber to amcl_pose topic in a different ROS_DOMAIN_ID
        # self._amcl_pose_sub = self.create_subscription(
        #     PoseStamped,
        #     "/amcl_pose",
        #     self.amcl_pose_callback,
        #     10
        # )

        self.get_logger().info("Follower navigation demo started")

    def localize(self):
        """
        Set the initial pose of the robot.
        """

        # Set the initial pose of the robot
        self._follower_initial_pose.header.frame_id = "map"
        self._follower_initial_pose.header.stamp = (
            self._follower_navigator.get_clock().now().to_msg()
        )
        self._follower_initial_pose.pose.position.x = 1.0
        self._follower_initial_pose.pose.position.y = 1.0
        self._follower_initial_pose.pose.position.z = 0.0
        self._follower_initial_pose.pose.orientation.x = 0.0
        self._follower_initial_pose.pose.orientation.y = 0.0
        self._follower_initial_pose.pose.orientation.z = 0.0
        self._follower_initial_pose.pose.orientation.w = 1.0
        self._follower_navigator.setInitialPose(self._follower_initial_pose)

    def navigate(self, x: float, y: float):
        """
        Navigate the robot to the goal (x, y).
        """

        self._follower_navigator.waitUntilNav2Active()  # Wait until Nav2 is active

        goal = self.create_pose_stamped(x, y, 0.0)

        self._follower_navigator.goToPose(goal)
        while not self._follower_navigator.isTaskComplete():
            feedback = self._follower_navigator.getFeedback()
            self.get_logger().info(f"Feedback: {feedback}")

        result = self._follower_navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info("Goal succeeded")
        elif result == TaskResult.CANCELED:
            self.get_logger().info("Goal was canceled!")
        elif result == TaskResult.FAILED:
            self.get_logger().info("Goal failed!")

    def follow_waypoints(self):
        self._follower_navigator.waitUntilNav2Active()  # Wait until Nav2 is active

        pose1 = self.create_pose_stamped(-0.979630, 0.262296, 0.0)
        pose2 = self.create_pose_stamped(4.039209, 2.856321, 0.0)
        pose3 = self.create_pose_stamped(-3.941751, 8.195021, 0.0)
        waypoints = [pose1, pose2, pose3]
        self._follower_navigator.followWaypoints(waypoints)

        while not self._follower_navigator.isTaskComplete():
            feedback = self._follower_navigator.getFeedback()
            self.get_logger().info(f"Feedback: {feedback}")

        result = self._follower_navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info("Goal succeeded")
        elif result == TaskResult.CANCELED:
            self.get_logger().info("Goal was canceled!")
        elif result == TaskResult.FAILED:
            self.get_logger().info("Goal failed!")

    def create_pose_stamped(self, x: float, y: float, yaw: float) -> PoseStamped:
        """
        Create and return a PoseStamped message.
        """

        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = self._follower_navigator.get_clock().now().to_msg()
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = 0.0

        # Convert yaw to quaternion
        q_x, q_y, q_z, q_w = tf_transformations.quaternion_from_euler(0.0, 0.0, yaw)
        goal.pose.orientation.x = q_x
        goal.pose.orientation.y = q_y
        goal.pose.orientation.z = q_z
        goal.pose.orientation.w = q_w

        goal.pose.orientation.x = q_x
        goal.pose.orientation.y = q_y
        goal.pose.orientation.z = q_z
        goal.pose.orientation.w = q_w
        return goal
    
    
def main(args=None):
    rclpy.init(args=args)
    follower_node = FollowerNavigationDemoInterface()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(follower_node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        follower_node.get_logger().info("Keyboard Interrupt (SIGINT) detected")
    finally:
        follower_node.destroy_node()

        rclpy.shutdown()