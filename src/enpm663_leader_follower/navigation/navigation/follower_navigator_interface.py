from geometry_msgs.msg import PoseStamped, PoseArray, TransformStamped, Quaternion, PoseWithCovarianceStamped
from navigation.robot_navigator_interface import BasicNavigator, TaskResult
import rclpy
from rclpy.parameter import Parameter
from rclpy.node import Node
import tf_transformations
from tf2_ros import TransformBroadcaster
from final_project.tf_broadcaster import TFBroadcaster
from final_project.utilities import pose_info, euler_from_quaternion
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf2_ros
import math
from std_srvs.srv import Trigger 
import time
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

class Color:
    """
    Define different colors for printing messages to terminals.
    """
    RED = "\033[91m"
    GREEN = "\033[92m"
    YELLOW = "\033[93m"
    BLUE = "\033[94m"
    CYAN = "\033[96m"  # Adding Cyan color
    MAGENTA = "\033[95m"  # Adding Magenta color
    END = "\033[0m"

class FollowerNavigationDemoInterface(Node):
    """
    Example of a class that uses the BasicNavigator class to navigate the robot.
    """
    def __init__(self, node_name="follower_navigation", namespace="follower"):
        super().__init__(node_name)

        # Declare the follower parameter - to determine the task of the follower robot
        self.declare_parameter("follower", "init")
        # get the parameter value
        self._task_param = (self.get_parameter("follower").get_parameter_value().string_value)

        # Since we are using Gazebo, we need to set the use_sim_time parameter to True
        self._sim_time = Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, True)
        self.set_parameters([self._sim_time])

        # Navigator
        self._follower_navigator = BasicNavigator(node_name, namespace)
        self._move_to_goal = True
        # Initial pose
        self._follower_initial_pose = PoseStamped()

        # Task the follower robot should perform - goal and waypoints are used for testing
        if self._task_param == "init":
            self.localize()
        elif self._task_param == "goal":
            self.localize()
            self.navigate(-9.752330, -0.288182)
        elif self._task_param == "waypoints":
            self.localize()
            self.follow_waypoints()

        # -------------------------------
        # Subscription to leader location
        # -------------------------------
        # QoS for Subscriber
        amcl_pose_qos = QoSProfile(
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
        )
        # Amcl Subscriber
        self._amcl_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            "leader/amcl_pose",
            self.amcl_pose_callback,
            amcl_pose_qos
        )
    
        # -------------------
        # Broadcaster
        # -------------------
        # Parent frame -> world
        self._part_parent_frame = "world"
        # Child frame -> leader/map because leader position data is published to leader/amcl_pose 
        self._part_frame = "leader/map"
        # Broadcaster node
        self._leader_tf_broadcaster = TransformBroadcaster(self)        

        self.get_logger().info("Broadcaster/Listener demo started")


    def amcl_pose_callback(self, msg):
        """
        AMCL subscription callback to transform data to broadcast for listener
        Msg data format: PoseWithCovarianceStamped
        Msg data is then formatted for input to utilities function pose_info for broadcaster
        """
        # If no parts are detected, return
        self.get_logger().info("Received amcl pose")
        self.initial_pose_received = True

        # Format position and orientation for navigation
        orientation_list = Quaternion()
        orientation_list.x = msg.pose.pose.orientation.x
        orientation_list.y = msg.pose.pose.orientation.y
        orientation_list.z = msg.pose.pose.orientation.z
        orientation_list.w = msg.pose.pose.orientation.w
        
        self.navigate(float(msg.pose.pose.position.x), float(msg.pose.pose.position.y))


    def _listener_cb(self):
        """
        Callback function for the listener timer.
        """
        try:
            # Get the transform between frames
            transform = self._tf_buffer.lookup_transform(
                "world", self._part_frame, rclpy.time.Time()
            )

            # Print to follower node terminal window
            output = "\n"
            output += "=" * 50 + "\n"
            output += f"Transform from {self._part_frame} to world \n"
            output += f"Translation: x: {transform.transform.translation.x}, y: {transform.transform.translation.y}, z: {transform.transform.translation.z}\n"
            output += "=" * 50 + "\n"
            self.get_logger().info(Color.GREEN + output + Color.END)

            # If robot not at goal location, navigate to goal
            if self._move_to_goal == True:
                self.get_logger().info("Navigating to leader position.")
                self.navigate(transform.transform.translation.x, transform.transform.translation.y)
            # If robot is at goal, do not move
            else:
                self.get_logger().info("Follower reached leader. Follower idle.")
                time.sleep(1) # wait 1 sec and reset to freely-move
                self._move_to_goal = True

        except TransformException as ex:
            # If robot is not at goal, report failure and wait for new goal provided by listener
            if self._move_to_goal == True:
                self.get_logger().fatal(f"Could not get transform between world and {self._part_frame}: {str(ex)}")
            else:
                # If robot is at goal, wait until new goal provided to listener 
                self. get_logger().info("Follower Idle")

    def localize(self):
        """
        Set the initial pose of the robot.
        """
        # Set the initial pose of the robot
        self._follower_initial_pose.header.frame_id = "map"  # initialize the robots in the map to create a map frame
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
        Navigate the robot to the goal (x, y) floating points.
        """
        self._follower_navigator.waitUntilNav2Active()  # Wait until Nav2 is active
        goal = self.create_pose_stamped(x, y, 0.0)
        self._follower_navigator.goToPose(goal)
        while not self._follower_navigator.isTaskComplete():
            feedback = self._follower_navigator.getFeedback()
            #self.get_logger().info(f"Feedback: {feedback}")

        # Report result of navigation
        result = self._follower_navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info("Goal succeeded")
            self._move_to_goal = False
        elif result == TaskResult.CANCELED:
            self.get_logger().info("Goal was canceled!")
            self._move_to_goal = True
        elif result == TaskResult.FAILED:
            self.get_logger().info("Goal failed!")
            self._move_to_goal = True

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