import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import struct
import zivid
import ros2_numpy as rnp
import array
import sys

from sensor_msgs.msg import PointCloud2, PointField, Image
from std_srvs.srv import Trigger
from std_msgs.msg import Header
from std_srvs.srv import Trigger


class GetMockZividFrame(Node):

    def __init__(self):
        super().__init__('mock_zivid_camera')    # create a node
        self.srv = self.create_service(Trigger, 'capture_mock_zivid_frame', self.mock_zivid_camera_callback)  # create service server
        self.get_logger().info('capture_mock_zivid_frame Service is ready.')

        # Publishers for PointCloud2 and Image
        self.pointcloud_publisher = self.create_publisher(PointCloud2, 'pointcloud', 10)
        self.image_publisher = self.create_publisher(Image, 'image_color', 10)

        # Declare the parameter , set default -> can be passed from a launch file
        self.declare_parameter('sim_scan_path', '/home/fanuc_m16ib/ros2_ws/src/zivid_simulation/data/scan2.zdf')
        self.simScanPath = self.get_parameter('sim_scan_path').get_parameter_value().string_value


    def mock_zivid_camera_callback(self, request, response):
        # Zivid initialization and loading the point cloud data
        with zivid.Application() as app:
            frame = zivid.Frame(self.simScanPath)
            point_cloud = frame.point_cloud()

            # Extract xyz, rgb, and normals
            xyz_ = point_cloud.copy_data("xyz")
            points = np.nan_to_num(xyz_)  # Handle NaN values           
            rgb = point_cloud.copy_data("bgra")[:, :, :3]  # Extract RGB channels

            # Publish PointCloud2
            self.publish_pointcloud(points, rgb)

            # Publish RGB image
            self.publish_image(rgb)

            # Return success response
            response.success = True
            response.message = 'Zivid point cloud and RGB image published'
        return response


    def publish_pointcloud(self, points, rgb):

        # Example input: a height x width x 3 array with float32 type for RGB
        height = points.shape[0]
        width = points.shape[1]

        # Number of points
        npoints = height * width

        # Reshape original array to a shape suitable for the structured array
        points_flatten = points.reshape(npoints, 3)
        rgb_flatten = rgb.reshape(npoints, 3)

        # Create the structured array with a single xyz field and a single rgb field
        points_arr = np.zeros((npoints,), dtype=[
            ('xyz', np.float32, (3,)),  # Combined field for x, y, z
            ('rgb', np.uint8, (3,))      # Combined field for r, g, b
        ])

        # Populate the structured array
        points_arr['xyz'] = points_flatten 
        points_arr['rgb'] = rgb_flatten

        # Convert the numpy array to a PointCloud2 message using ros2_numpy
        pointcloud_msg = self.array_to_point_cloud2(points_arr, 'zivid_optical_frame')

        # Publish the message
        self.pointcloud_publisher.publish(pointcloud_msg)
        self.get_logger().info('PointCloud2 Data published')



    def publish_image(self, rgb):
        # Prepare Image message
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'zivid_optical_frame'

        # Create an Image message
        img_msg = Image()
        img_msg.header = header
        img_msg.height = rgb.shape[0]
        img_msg.width = rgb.shape[1]
        img_msg.encoding = 'bgr8'
        img_msg.is_bigendian = False
        img_msg.step = 3 * rgb.shape[1]
        img_msg.data = rgb.tobytes()  # Convert the RGB data to a byte array

        self.image_publisher.publish(img_msg)
        self.get_logger().info('RGB image published')
    


    def array_to_point_cloud2(self, np_array, frame_id='map'):
        """
        Convert a numpy array to a PointCloud2 message. The numpy array must have a "xyz" field
        and can optionally have a "rgb" field and a "intensity" field.
        """
        # Check if the "rgb" field is present
        rgb_flag = False

        # Create the PointCloud2 message
        msg = PointCloud2()
        msg.header.frame_id = frame_id
        msg.header.stamp = self.get_clock().now().to_msg()  
        msg.height = 1
        msg.width = np_array["xyz"].shape[0]
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)]

        if rgb_flag:
            msg.fields.append(PointField(name='rgb', offset=12,
                            datatype=PointField.UINT32, count=1))

        msg.is_bigendian = sys.byteorder != 'little'

        # Check if message is dense
        msg.is_dense = not np.isnan(np_array["xyz"]).any()

        # Calculate the point_step and row_step
        if rgb_flag:
            msg.point_step = 16
        else:
            msg.point_step  =12

        msg.row_step = msg.point_step * msg.width

        # The PointCloud2.data setter will create an array.array object for you if you don't
        # provide it one directly. This causes very slow performance because it iterates
        # over each byte in python.
        # Here we create an array.array object using a memoryview, limiting copying and
        # increasing performance.

        if rgb_flag:
            memory_view = memoryview(np.hstack((np_array["xyz"].astype(np.float32).tobytes(
            ), np_array["rgb"].astype(np.uint32).tobytes())))        
        else:
            memory_view = memoryview(np_array["xyz"].astype(np.float32).tobytes())


        if memory_view.nbytes > 0:
            array_bytes = memory_view.cast("B")
        else:
            # Casting raises a TypeError if the array has no elements
            array_bytes = b""

        as_array = array.array("B")
        as_array.frombytes(array_bytes)
        msg.data = as_array

        return msg
    


def main(args=None):
    rclpy.init(args=args)
    my_mock_zivid_camera = GetMockZividFrame()
    rclpy.spin(my_mock_zivid_camera)
    rclpy.shutdown()

if __name__ == '__main__':
    main()