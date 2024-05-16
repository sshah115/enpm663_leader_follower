from geometry_msgs.msg import PoseStamped
from navigation.robot_navigator_interface import BasicNavigator, TaskResult
import rclpy
from rclpy.parameter import Parameter
from rclpy.node import Node
import tf_transformations
from std_srvs.srv import Trigger 

class LeaderNavigationDemoInterface(Node):
    """
    Example of a class that uses the BasicNavigator class to navigate the robot.
    """
    def __init__(self, node_name="leader_navigation", namespace="leader"):
        super().__init__(node_name)

        # Declare the leader parameter
        # This parameter is used to determine the task of the leader robot
        self.declare_parameter("leader", "waypoints")
        # get the parameter value
        self._task_param = (
            self.get_parameter("leader").get_parameter_value().string_value
        )
        # Since we are using Gazebo, we need to set the use_sim_time parameter to True
        self._sim_time = Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, True)
        self.set_parameters([self._sim_time])
        # Navigator
        self._leader_navigator = BasicNavigator(node_name, namespace)
        # Initial pose
        self._leader_initial_pose = PoseStamped()

        # Added - current position
        self._position = self._leader_initial_pose.pose.position
        
        # Task the leader robot should perform
        if self._task_param == "init":
            self.localize()
        elif self._task_param == "goal":
            self.localize()
            self.navigate(-0.872918, -7.414840)
        elif self._task_param == "waypoints":
            self.localize()
            self.follow_waypoints()

        # subscriber to amcl_pose topic in a different ROS_DOMAIN_ID
        # Kept - Required to get accurate location of the leader robot
        self._amcl_pose_sub = self.create_subscription(
            PoseStamped,
            "/amcl_pose",
            self.amcl_pose_callback,
            10
        )

        # Added
        # publisher to amcl_pose topic for trigger request
        self._amcl_pose_pub = self.create_publisher(
            PoseStamped,
            "/leader_coordinates",
            10
        )
        # Asynchronous leader_location service
        self._leader_location_srv = self.create_service(Trigger, "leader_location", self.trigger_response)

        self.get_logger().info("Leader navigation demo started")


    # Added - Carissa
    def trigger_response(self,request,response):
        # position.x and position.y
        self._amcl_pose_pub.publish(self._position) 
        response.success = True
        response.message = "Server Processed Client Request: Leader Location Trigger"
        return response

    # Added - For the amcl_pose subscription and publisher to work this is the dependency
    def amcl_pose_callback(self, msg):
        self._position = msg.pose.pose.position
        self.get_logger().info(f"Position: x={self._position.x}, y={self._position.y}")


    def localize(self):
        """
        Set the initial pose of the robot.
        """

        # Set the initial pose of the robot
        self._leader_initial_pose.header.frame_id = "map"
        self._leader_initial_pose.header.stamp = (
            self._leader_navigator.get_clock().now().to_msg()
        )
        self._leader_initial_pose.pose.position.x = 1.0
        self._leader_initial_pose.pose.position.y = 2.0
        self._leader_initial_pose.pose.position.z = 0.0
        self._leader_initial_pose.pose.orientation.x = 0.0
        self._leader_initial_pose.pose.orientation.y = 0.0
        self._leader_initial_pose.pose.orientation.z = 0.0
        self._leader_initial_pose.pose.orientation.w = 1.0
        self._leader_navigator.setInitialPose(self._leader_initial_pose)

    def navigate(self, x: float, y: float):
        """
        Navigate the robot to the goal (x, y).
        """

        self._leader_navigator.waitUntilNav2Active()  # Wait until Nav2 is active

        goal = self.create_pose_stamped(x, y, 0.0)

        self._leader_navigator.goToPose(goal)
        while not self._leader_navigator.isTaskComplete():
            feedback = self._leader_navigator.getFeedback()
            #self.get_logger().info(f"Feedback: {feedback}")

        result = self._leader_navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info("Goal succeeded")
        elif result == TaskResult.CANCELED:
            self.get_logger().info("Goal was canceled!")
        elif result == TaskResult.FAILED:
            self.get_logger().info("Goal failed!")

    def follow_waypoints(self):
        self._leader_navigator.waitUntilNav2Active()  # Wait until Nav2 is active

        pose1 = self.create_pose_stamped(-0.872918, -7.414839, 0.0)
        pose2 = self.create_pose_stamped(-9.752331, -0.288182, 0.0)
        pose3 = self.create_pose_stamped(-3.941751, 8.195021, 0.0)
        waypoints = [pose1, pose2, pose3]
        self._leader_navigator.followWaypoints(waypoints)

        while not self._leader_navigator.isTaskComplete():
            feedback = self._leader_navigator.getFeedback()
            #self.get_logger().info(f"Feedback: {feedback}")

        result = self._leader_navigator.getResult()
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
        goal.header.stamp = self._leader_navigator.get_clock().now().to_msg()
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = 0.0

        # Convert yaw to quaternion
        q_x, q_y, q_z, q_w = tf_transformations.quaternion_from_euler(0.0, 0.0, yaw)
        goal.pose.orientation.x = q_x
        goal.pose.orientation.y = q_y
        goal.pose.orientation.z = q_z
        goal.pose.orientation.w = q_w

        return goal


def main(args=None):
    rclpy.init(args=args)
    leader_node = LeaderNavigationDemoInterface()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(leader_node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        leader_node.get_logger().info("Keyboard Interrupt (SIGINT) detected")
    finally:
        leader_node.destroy_node()

        rclpy.shutdown()