import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
import time
from typing import List
import tf_transformations as tft
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
from geometry_msgs.msg import TransformStamped
from rclpy.time import Time, Duration
from tf2_ros import TransformException, ConnectivityException, LookupException, ExtrapolationException

# service and action interfaces/ types
from moveit_wrapper.srv import MoveToPose, MoveToJointPosition, SetVelocity, String  #import the custom service interfaces from the wrapper package
from std_srvs.srv import Empty

# Affine py dependencies
from api_moveit_clients.transform import Affine
from api_moveit_clients.util import affine_to_pose


class ARMClient(Node):
     
    def __init__(self):
        super().__init__('arm_client_node')

        self.tf_buffer = Buffer()
        self.transform_listener = TransformListener(self.tf_buffer, self)

        # init all needed clients (connect to the servers from moveit_wrapper package --> Wrap MoveIt C++ Interface to Python, because MoveIt Python API has only core capabilities)
        self.move_lin_cli = self.create_client(MoveToPose, "/move_to_pose_lin")    #connects to the services defined in the moveit_wrapper srvs
        while not self.move_lin_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("move_to_pose_lin service not available, waiting some more ...")
        self.get_logger().info("move_to_pose_lin service available")

        self.move_ptp_cli = self.create_client(MoveToPose, "/move_to_pose_ptp")
        while not self.move_ptp_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("move_to_pose_ptp service not available, waiting some more ...")
        self.get_logger().info("move_to_pose_ptp service available")

        self.move_joint_cli = self.create_client(MoveToJointPosition, "/move_to_joint_position")
        while not self.move_joint_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("move_to_joint_position service not available, waiting some more ...")
        self.get_logger().info("move_to_joint_position service available")

        self.reset_planning_group_cli = self.create_client(String, "/reset_planning_group")
        while not self.reset_planning_group_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("reset_planning_group service not available, waiting some more ...")
        self.get_logger().info("reset_planning_group service available")

        self.set_velocity_cli = self.create_client(SetVelocity, "/setVelocityScaling")
        while not self.set_velocity_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("setVelocityScaling service not available, waiting some more ...")
        self.get_logger().info("setVelocityScaling service available")

        self.clear_octomap_cli = self.create_client(Empty, "/clear_octomap")
        while not self.clear_octomap_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("clear_octomap service not available, waiting some more ...")
        self.get_logger().info("clear_octomap service available")

        self.home_position = [0.0,0.0,0.0,0.0,0.0,0.0]

    ##############################################################################################################################
    ##           this are the methods which can be called in your application which are communicating to the robot              ##
    ##############################################################################################################################
    def get_transform(self, from_frame_rel, to_frame_rel, affine=True):
        """
        string from_frame_rel: name of the source frame frame to transform from (child)
        string to_frame_rel: name of the target frame to transform into (parent)

        Returns:
        --------
        Affine: transformation matrix from robot base to target position and orientation (4x4 numpy array, can directly passed in motion planning)
        or
        geometry_msgs/TransformStamped: transformation between the two frames if affine=False
        """
        # Calculate the transform between the robot base frame and the target frame
        counter = 0
        has_transform = False
        transform = None
        while (not has_transform) and counter < 200:
            rclpy.spin_once(self)

            counter += 1
    
            #if counter % 10 == 0:
            #    self.get_logger().warn(f'Try to find transform {to_frame_rel} to {from_frame_rel} {counter} times... try again...')

            try:
                has_transform = self.tf_buffer.can_transform(to_frame_rel, from_frame_rel, time=Time(), timeout=Duration(seconds=0.01))
                if has_transform:
                    transform = self.tf_buffer.lookup_transform(to_frame_rel, from_frame_rel, time=Time(), timeout=Duration(seconds=0.01))   # takes latest available tf in buffer
                    trans = transform.transform
                    if affine:
                        transform = Affine(
                            [trans.translation.x, trans.translation.y, trans.translation.z],
                            [trans.rotation.x, trans.rotation.y, trans.rotation.z, trans.rotation.w])
                
            except (TransformException, LookupException, ConnectivityException, ExtrapolationException) as ex:
                self.get_logger().error(f'Exception during transform {from_frame_rel} to {to_frame_rel} after {counter} tries... {ex}')                
                transform = None
            
        if transform != None:    
            self.get_logger().info(f'Found transform {from_frame_rel} to {to_frame_rel} after {counter} tries') 
        else:
            self.get_logger().error(f'Could not transform {from_frame_rel} to {to_frame_rel} after {counter} tries...try again and check passed frame ids...')
        return transform

    

    def reset_planning_group(self, planning_group) -> bool:
        """
        string planning_group: name of the planning group of the arm (default: igus_6dof)

        Returns
        -------
        bool success

        """
        req = String.Request()
        req.data = planning_group
        future = ARMClient.send_service_request(req, self.reset_planning_group_cli)
        response = self.wait_for_service_response(future)
        return response.success

    def setVelocity(self, fraction) -> bool:
        """
        float: velocity caling coeffinet relative to joint speed limits (0.01 ... 0.5 recommendet)

        Returns
        -------
        bool success

        """
        req = SetVelocity.Request()
        req.velocity_scaling = fraction
        future = ARMClient.send_service_request(req, self.set_velocity_cli)
        response = self.wait_for_service_response(future)
        if not response.success:
            self.get_logger().error('Could not set velocity scaling')
        return response.success
    
    def home(self) -> bool:
        """

        Returns
        -------
        bool success

        """
        return self.ptp_joint(self.home_position)

    # planning in cartesian space, plans around obstacles and octomap
    def ptp(self, pose: Affine) -> bool:
        """
        cartesian goal pose: affine transformation matrix format (can be converted from x,y,z, r,p,y or x,y,z and quaternion)

        Returns
        -------
        bool success

        """
        req = MoveToPose.Request()
        req.pose = affine_to_pose(pose)
        future = ARMClient.send_service_request(req, self.move_ptp_cli)
        response = self.wait_for_service_response(future)

        print("Trajectory: ", len(response.trajectory.points))

        if not response.success:
            self.get_logger().error('Could not move to cartesian position due to detected error in ptp path or position is out of the workspace')
        return response.success

    # plan in joint_space, move to joint positions directly, but dont plan around obstacles
    def ptp_joint(self, joint_positions: List[float]) -> bool:
        """
        joint space goal pose: list of goal joint states (rad)

        Returns
        -------
        bool success

        """
        req = MoveToJointPosition.Request()
        req.joint_position = joint_positions
        future = ARMClient.send_service_request(req, self.move_joint_cli)
        response = self.wait_for_service_response(future)
        if not response.success:
            self.get_logger().error('Could not move to joint position due to detected error')
        return response.success

    # plan trajectory along cartesian lin path, but dont plan around obstacles
    def lin(self, pose: Affine) -> bool:
        """
        cartesian goal pose: affine transformation matrix format (can be converted from x,y,z, r,p,y or x,y,z and quaternion)

        Returns
        -------
        bool success

        """
        req = MoveToPose.Request()
        req.pose = affine_to_pose(pose)
        future = ARMClient.send_service_request(req, self.move_lin_cli)
        response = self.wait_for_service_response(future)
        if not response.success:
            self.get_logger().error('Could not move to cartesian position due to detected error in linear path or position is out of the workspace')
        return response.success


    # clear octomap in planning scene, only use current pointcloud for detecting collision geometry in the planning scene
    def clear_octomap(self) -> None:
        """
        Returns
        -------
        None
        """
        req = Empty.Request()
        future = ARMClient.send_service_request(req, self.clear_octomap_cli)
        self.wait_for_service_response(future)
        return





    ##########################################################################################################################################################
    @staticmethod
    def send_service_request(request, client):
        future = client.call_async(request)
        return future
    
    @staticmethod
    def send_action_request(request, client):
        client.wait_for_server()
        future = client.send_goal_async(request)   # callback for ongoing feedback of action servers currently not implemented, buf could be handled here!

        return future

    def wait_for_service_response(self, future):
        while rclpy.ok():
            rclpy.spin_once(self)
            if future.done():
                try:
                    response = future.result()
                except Exception as e:
                    self.get_logger().info(
                        'Service call failed %r' % (e,))
                    return None
                else:
                    return response
                

    def wait_for_action_goal_acknowledgment(self, goal_future):
        # handle the process of sending the goal to the action server and receiving the server's acknowledgment
        # this future is completed when the action server processes the goal request
        while rclpy.ok():
            rclpy.spin_once(self)
            if goal_future.done():
                try:
                    goal_handle = goal_future.result()
                except Exception as e:
                    self.get_logger().info(f'Action goal send failed: {e}')
                    return None
                else:
                    if not goal_handle.accepted:
                        self.get_logger().info('Action goal rejected')
                    self.get_logger().info('Action goal accepted')
                    return goal_handle
                
    
    def wait_for_action_result(self, goal_handle):
        result_future = goal_handle.get_result_async()
        # used to handle the process of waiting for the action to complete and retrieving the final result
        # this future is completed when the action server finishes processing the action and sends back the final result
        while rclpy.ok():
            rclpy.spin_once(self)
            if result_future.done():
                try:
                    result = result_future.result()  # access result field from action definition
                except Exception as e:
                    self.get_logger().info(f'Action call failed: {e}')
                    return None
                else:
                    return result
                
    def destroy_node(self) -> None:
        self.destroy_node()
