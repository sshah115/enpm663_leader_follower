from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from navigation.robot_navigator_interface import BasicNavigator, TaskResult
import rclpy
from rclpy.parameter import Parameter
from rclpy.node import Node
import tf_transformations
from std_srvs.srv import Trigger 
 
import os
from ament_index_python.packages import get_package_share_directory
import yaml
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Pose, Quaternion, TransformStamped, PoseStamped
import PyKDL
from mage_msgs.msg import AdvancedLogicalCameraImage, Part
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
import copy


class LeaderNavigationDemoInterface(Node):
    """
    Example of a class that uses the BasicNavigator class to navigate the robot.
    """
    def __init__(self, node_name="leader_navigation", namespace="leader"):
        super().__init__(node_name)

        # Declare the leader parameter
        # This parameter is used to determine the task of the leader robot
        self.declare_parameter("leader", "goal")
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

        # Added
        qos_policy = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.itr = 0
        self.get_logger().info(f"Temp:")
        
            
        # Task the leader robot should perform
        if self._task_param == "init":
            self.localize()
        elif self._task_param == "goal":
            self.localize()
            self.navigate(-0.872918, -7.414840)
        elif self._task_param == "waypoints":
            #self.localize()
            self.extract_waypoints() # extracts wp lists to self._targets

            # --------Shail--------
            self._camera_dict = {}

            # Create callback groups - Async
            self.callback_group = {
                "_camera_callback_group": ReentrantCallbackGroup(),                
            }

            # # Define the subscriptions
            subscriptions = {
                "_camera1_subscription": (
                    "/mage/camera1/image",
                    AdvancedLogicalCameraImage,
                    lambda msg: self._camera_callback(msg, 1),
                ),
                "_camera2_subscription": (
                    "/mage/camera2/image",
                    AdvancedLogicalCameraImage,
                    lambda msg: self._camera_callback(msg, 2),
                ),
                "_camera3_subscription": (
                    "/mage/camera3/image",
                    AdvancedLogicalCameraImage,
                    lambda msg: self._camera_callback(msg, 3),
                ),
                "_camera4_subscription": (
                    "/mage/camera4/image",
                    AdvancedLogicalCameraImage,
                    lambda msg: self._camera_callback(msg, 4),
                ),
                "_camera5_subscription": (
                    "/mage/camera5/image",
                    AdvancedLogicalCameraImage,
                    lambda msg: self._camera_callback(msg, 5),
                ),
            }

            # # Create the subscriptions
            for attr, (topic, msg_type, callback) in subscriptions.items():
                setattr(
                    self,
                    attr,
                    self.create_subscription(
                        msg_type,
                        topic,
                        callback,
                        qos_profile=qos_policy,
                        callback_group=self.callback_group["_camera_callback_group"],
                    ),
                )
                self.localize() ### Why we are localizing the robot here

        # The following commented block was added for the location server, but removed becuase as per latest implementation it is redundant
        # Subscribe to amcl_pose to extract leaders location
        # self._amcl_pose_sub = self.create_subscription(
        #     PoseStamped,
        #     "/amcl_pose",
        #     self.amcl_pose_callback,
        #     10
        # )

        # Publisher for the leader location request
        # self.amcl_pose_pub = self.create_publisher(
        #     PoseStamped,
        #     "/leader_coordinates",
        #     5
        # )

        # self._leader_location_service = self.create_service(Trigger, "leader_location", self.trigger_response)


        self.get_logger().info("Leader navigation demo started")

    
    # Note - The following block was added for the server
    # def trigger_response(self, request, response):
    #     self._amcl_pose_pub.publis(self._position)
    #     response.success = True
    #     response.message = "Server processed client request: Leader location request triggered"
    #     return response

    # def amcl_pose_callback(self, msg):
    #     self._position.x = msg.pose.pose.position.x
    #     self._position.y = msg.pose.pose.position.y
    #     self.get_logger().info(f"Position: x={self._position.x}, y={self._position.y}")
    

    def extract_waypoints(self):
        """
        Function to extract waypoints from parameter file

        Args:
            param_file (str): Location of file with waypoints
        """
        param_file = os.path.join(
            get_package_share_directory("navigation"),
            "params",
            "waypoints.yaml",
        )

        self.get_logger().info("Leader navigation demo started")

        with open(param_file, "r") as f:
            coordinates = yaml.safe_load(f)
            targets = coordinates["leader_navigation"]["ros__parameters"]
        self._targets = targets 
        self.get_logger().info(f"Waypoints: {self._targets}")

    def _camera_callback(self, msg, camera_id):
        """
        Camera Callback is used to scan cameras and store data

        Args:
            msg: Image type from topic
            camera_id: Arg passed when subscription callback created
        """
        if camera_id not in self._camera_dict:
            self._camera_dict[camera_id] = {}
        part_pose = msg.part_poses
        if len(part_pose) > 0:
            for i in range(len(part_pose)):
                part = part_pose[i].part
                camera_pose = Pose()
                camera_pose.position.x = msg.sensor_pose.position.x
                camera_pose.position.y = msg.sensor_pose.position.y
                camera_pose.position.z = msg.sensor_pose.position.z
                camera_pose.orientation.x = msg.sensor_pose.orientation.x
                camera_pose.orientation.y = msg.sensor_pose.orientation.y
                camera_pose.orientation.z = msg.sensor_pose.orientation.z
                camera_pose.orientation.w = msg.sensor_pose.orientation.w

                detected_part_pose = Pose()
                detected_part_pose.position.x = part_pose[i].pose.position.x
                detected_part_pose.position.y = part_pose[i].pose.position.y
                detected_part_pose.position.z = part_pose[i].pose.position.z
                detected_part_pose.orientation.x = part_pose[i].pose.orientation.x
                detected_part_pose.orientation.y = part_pose[i].pose.orientation.y
                detected_part_pose.orientation.z = part_pose[i].pose.orientation.z
                detected_part_pose.orientation.w = part_pose[i].pose.orientation.w

                part_world_pose = self._multiply_pose(camera_pose, detected_part_pose)

                self._camera_dict[camera_id].update(
                    {i: {"part": part, "pose": part_world_pose}}
                )
        if len(self._camera_dict)>0 and self.itr < 5:
            self.itr += 1
            if self.itr == 5:
                temp = self._camera_dict
                self._camera_dict.clear()
                self._camera_dict = temp
                self.follow_waypoints()
            

    def _multiply_pose(self, pose1: Pose, pose2: Pose) -> Pose:
        """
        Use KDL to multiply two poses together.
        Args:
            pose1 (Pose): Pose of the first frame
            pose2 (Pose): Pose of the second frame
        Returns:
            Pose: Pose of the resulting frame
        """

        orientation1 = pose1.orientation
        frame1 = PyKDL.Frame(
            PyKDL.Rotation.Quaternion(
                orientation1.x, orientation1.y, orientation1.z, orientation1.w
            ),
            PyKDL.Vector(pose1.position.x, pose1.position.y, pose1.position.z),
        )

        orientation2 = pose2.orientation
        frame2 = PyKDL.Frame(
            PyKDL.Rotation.Quaternion(
                orientation2.x, orientation2.y, orientation2.z, orientation2.w
            ),
            PyKDL.Vector(pose2.position.x, pose2.position.y, pose2.position.z),
        )

        frame3 = frame1 * frame2

        # return the resulting pose from frame3
        pose = Pose()
        pose.position.x = frame3.p.x()
        pose.position.y = frame3.p.y()
        pose.position.z = frame3.p.z()

        # Get Quaternion
        q = frame3.M.GetQuaternion()
        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]

        return pose

    def create_waypoint_list(self):
        waypoints = []
        for key in sorted(self._targets.keys()):
            part_desc = self._targets[key]
            color = part_desc['color'].upper()
            part_type = part_desc['type'].upper()
            self.get_logger().info(f"Starting")
            world_pose = self.find_part_pose_in_world(color, part_type)
            if world_pose:
                waypoints.append(world_pose)
        return waypoints

    def find_part_pose_in_world(self, color, part_type):  
        # Use the constants defined in Part message for comparison
        color_value = getattr(Part, color.upper(), None)
        type_value = getattr(Part, part_type.upper(), None)

        if color_value is None or type_value is None:
            self.get_logger().error(f"Invalid color or type: {color}, {part_type}")
            return None

        for camera_id, parts in self._camera_dict.items():
            for part_info in parts.values():
                part = part_info["part"]
                self.get_logger().info(f"Checking part: color={part.color}, type={part.type}")
                if part.color == color_value and part.type == type_value:
                    return self.create_pose_stamped_from_pose(part_info["pose"])
        return None

    def create_pose_stamped_from_pose(self, pose):
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = "map"
        pose_stamped.header.stamp = self._leader_navigator.get_clock().now().to_msg()
        pose_stamped.pose = pose
        return pose_stamped


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

        # pose1 = self.create_pose_stamped(-0.872918, -7.414839, 0.0)
        # pose2 = self.create_pose_stamped(-9.752331, -0.288182, 0.0)
        # pose3 = self.create_pose_stamped(-3.941751, 8.195021, 0.0)
        # waypoints = [pose1, pose2, pose3]
        waypoints = self.create_waypoint_list()
        self._leader_navigator.followWaypoints(waypoints)

        while not self._leader_navigator.isTaskComplete():
            feedback = self._leader_navigator.getFeedback()
            self.get_logger().info(f"Feedback: {feedback}")

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
