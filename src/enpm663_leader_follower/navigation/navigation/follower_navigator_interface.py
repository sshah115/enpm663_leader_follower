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

        amcl_pose_qos = QoSProfile(
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        
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

        # -------------------
        # Trigger Service Client
        # -------------------
        # Subscriber for leader robot location data
        #self._amcl_pose_sub = self.create_subscription(
        #    PoseStamped,
        #    "/amcl_pose",
        #    self.amcl_pose_callback,
        #    10
        #)
        # TODO clean up subscription
        # BasicNavigator has amcl_pose already defined:
        # self.localization_pose_sub

        # Asynchronous trigger leader location client
        #self._leader_location_client = self.create_client(
        #    srv_type=Trigger,
        #    srv_name="leader_location"
        #)   
        
        # -------------------
        # Broadcaster
        # -------------------
        # Parent frame
        self._part_parent_frame = "follower/camera_rgb_frame"
        # Child frame
        self._part_frame = "follower/camera_rgb_optical_frame"
        # Broadcaster node
        self._leader_tf_broadcaster = TransformBroadcaster(self)
        #self._aruco_tf_broadcaster = TransformBroadcaster(self)

        #self._mutex_cbg1 = MutuallyExclusiveCallbackGroup()
        self._mutex_cbg2 = MutuallyExclusiveCallbackGroup()

        # TODO professor said we dont need to follow the aruco marker anymore
        # subscriber for aruco marker pose
        #self._aruco_pose_sub = self.create_subscription(
        #    PoseArray,
        #    "/aruco_poses",
        #    self.aruco_poses_cb,
        #    10,
        #    callback_group=self._mutex_cbg1,
        #)
        # overwrite robot_navigator_interface subscription
        self.localization_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            "leader/follower_amcl_pose",
            self._amclPoseCallback,
            10,
        )

        # -------------------
        # Listener
        # -------------------
        # Create a transform buffer and listener
        #self._tf_buffer = Buffer()
        #self._tf_listener = TransformListener(self._tf_buffer, self)

        # Listen to the transform between frames periodically
        #self._listener_timer = self.create_timer(0.5, self._listener_cb, callback_group=self._mutex_cbg2)
        self.get_logger().info("Broadcaster/Listener demo started")


    def _amclPoseCallback(self, msg):

        # If no parts are detected, return
        if len(msg.poses) == 0:
            self.get_logger().warn("No parts detected by the camera")
            return
        else:
            self.debug("Received amcl pose")
            self.initial_pose_received = True

            # Decode PoseWithCovarianceStamped msg into position/orientation
            position_list=[msg.pose.position.x, msg.pose.position.y]
            r,p,y = euler_from_quaternion(msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w)
            rpy_list = [r, p, y]
            navigate(msg.pose.position.x, msg.pose.position.y)
            # Send tf transforms 
            #self.broadcast(
            #    self._part_parent_frame, self._part_frame, pose=pose_info(position_list,rpy_list)
            #)

        return

    #def amcl_pose_callback(self,msg):
    #    # Navigate to leader position sent on amcl_pose
    #    self. navigate(msg.pose.position.x,msg.pose.position.y)
    #    output = "\n"
    #    output += "=" * 50 + "\n"
    #    output += f"Navigating to amcl_pose location! \n"
    #    output += f"Translation: x: {msg.pose.position.x}, y: {msg.pose.position.y} \n"
    #    output += "=" * 50 + "\n"
    #    self.get_logger().info(Color.GREEN + output + Color.END)

    def aruco_poses_cb(self,msg: PoseArray):
        """
        Callback function for the aruco pose subscription to broadcast the pose
        """

        # If no parts are detected, return
        if len(msg.poses) == 0:
            # self.get_logger().warn("No parts detected by the camera")
            return
        else:

            # silliest way of getting position and orientation out of PoseArray msg
            temp = (str(msg.poses)).split("(")
            temp2 = (str(temp)).split(")")
            temp3 = (str(temp2)).split(",")
            position=[temp3[2],temp3[3],temp3[4]]
            position_list=[]
            for each in position:
                number=each.split("=")
                position_list.append(float(number[1][:10]))
            orientation=[temp3[7],temp3[8],temp3[9],temp3[10]]
            orientation_list=[]
            for each in orientation:
                number=each.split("=")
                orientation_list.append(float(number[1][:10]))

            # Must convert the quaternionTransformStamped to euler for pose formatting 
            aruco_quaternion = Quaternion()
            aruco_quaternion.x = orientation_list[0]
            aruco_quaternion.y = orientation_list[1]
            aruco_quaternion.z = orientation_list[2]
            aruco_quaternion.w = orientation_list[3]
            r,p,y = euler_from_quaternion(aruco_quaternion)
            rpy_list = [r, p, y]

            # Print the Aruco Marker Pose detected
            output = "\n"
            output += "=" * 50 + "\n"
            output += "Part detected\n"
            output += f"Position: x: {position_list[0]}, y: {position_list[1]}, z: {position_list[2]}\n"
            output += f"Orientation: x: {orientation_list[0]}, y: {orientation_list[1]}, z: {orientation_list[2]}, w: {orientation_list[3]}\n"
            output += "=" * 50 + "\n"
            self.get_logger().info(Color.YELLOW + output + Color.END)

            # Send tf transforms 
            self.broadcast(
                self._part_parent_frame, self._part_frame, pose=pose_info(position_list,rpy_list)
            )

    def broadcast(self, parent, child, pose):

        """
        Build a transform message and broadcast it.

        Args:
            parent (str): Parent frame.
            child (str): Child frame.
            pose (geometry_msgs.msg.Pose): Pose of the child frame with respect to the parent frame.

        """
        transform_stamped = TransformStamped()
        transform_stamped.header.stamp = self.get_clock().now().to_msg()
        transform_stamped.header.frame_id = parent
        transform_stamped.child_frame_id = child

        transform_stamped.transform.translation.x = pose.position.x
        transform_stamped.transform.translation.y = pose.position.y
        transform_stamped.transform.translation.z = pose.position.z
        transform_stamped.transform.rotation.x = pose.orientation.x
        transform_stamped.transform.rotation.y = pose.orientation.y
        transform_stamped.transform.rotation.z = pose.orientation.z
        transform_stamped.transform.rotation.w = pose.orientation.w

        output = "\n"
        output += "=" * 50 + "\n"
        output += f"Broadcasting transform between {parent} and {child}\n"
        output += f"Translation: x: {pose.position.x}, y: {pose.position.y}, z: {pose.position.z}\n"
        output += f"Rotation: x: {pose.orientation.x}, y: {pose.orientation.y}, z: {pose.orientation.z}, w: {pose.orientation.w}\n"
        output += "=" * 50 + "\n"
        self.get_logger().info(Color.CYAN + output + Color.END)

        self._aruco_tf_broadcaster.sendTransform(transform_stamped)

    #def _listener_cb(self):
    #    """
    #    Callback function for the listener timer.
    #    """
    #    try:
            # Get the transform between frames
            #transform = self._tf_buffer.lookup_transform(
            #    "world", self._part_frame, rclpy.time.Time()
            #)
            #euler = euler_from_quaternion(transform.transform.rotation)
            # Print to follower node terminal window
            #output = "\n"
            #output += "=" * 50 + "\n"
            #output += f"Transform from {self._part_frame} to world \n"
            #output += f"Translation: x: {transform.transform.translation.x}, y: {transform.transform.translation.y}, z: {transform.transform.translation.z}\n"
            #output += f"Rotation: rpy: {euler[0]}, {euler[1]}, {euler[2]}\n"
            #output += "=" * 50 + "\n"
            #self.get_logger().info(Color.GREEN + output + Color.END)
            # If robot not at goal location, update goal with current aruco location
            #if self._move_to_goal == True:
            #    self.get_logger().info("****************** NAVIGATING ******************")
            #    self.navigate(transform.transform.translation.x, transform.transform.translation.y)
            ## If robot is at goal, do not move
            #else:
            ##    self.get_logger().info("******** GOAL SUCCEEDED. FOLLOWER IDLE ********")
             #   time.sleep(5) # wait 5 sec and reset to freely-move
             #   self._move_to_goal = True

        #except TransformException as ex:
        #    # If robot is not at goal and/or if robot can't see Aruco marker
        #    if self._move_to_goal == True:
        #        self.get_logger().fatal(
        #            f"Could not get transform between world and {self._part_frame}: {str(ex)}"
        #        )
        #        # Request leader robot location
        ##        #self. get_logger().info("****** Can't see Leader! Requesting Location! *****")
         #       #self.trigger_request
         #   else:
         #       # If robot is at goal, but can't see Aruco marker, request leader robot location
         #       self. get_logger().info("****** Goal Reached. Follower Robot IDLE! *****")
         #       #self.trigger_request#

    #def trigger_request(self):
    #    # Trigger Leader Robot Location
    #    request = Trigger.Request()
    #    future = self._leader_location_client.call_async(request)
    #    future.add_done_callback(self.leader_location_cb)

    #def leader_location_cb(self, future):
    #    try:
    #        response = future.result()
    #        self.get_logger().info(f"Response: {response.message}")
    #        if response.success:
    #            self. get_logger().info("****** Trigger successful! *****")
    #    except Exception as e:
    #        self.get_logger().error(f"Service call failed: {str(e)}")

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
        Navigate the robot to the goal (x, y).
        """

        self._follower_navigator.waitUntilNav2Active()  # Wait until Nav2 is active

        goal = self.create_pose_stamped(x, y, 0.0)

        self._follower_navigator.goToPose(goal)
        while not self._follower_navigator.isTaskComplete():
            feedback = self._follower_navigator.getFeedback()
            #self.get_logger().info(f"Feedback: {feedback}")

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

    def follow_waypoints(self):
        self._follower_navigator.waitUntilNav2Active()  # Wait until Nav2 is active

        pose1 = self.create_pose_stamped(-0.979630, 0.262296, 0.0)
        pose2 = self.create_pose_stamped(4.039209, 2.856321, 0.0)
        pose3 = self.create_pose_stamped(-3.941751, 8.195021, 0.0)
        waypoints = [pose1, pose2, pose3]
        self._follower_navigator.followWaypoints(waypoints)

        while not self._follower_navigator.isTaskComplete():
            feedback = self._follower_navigator.getFeedback()
            #self.get_logger().info(f"Feedback: {feedback}")

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