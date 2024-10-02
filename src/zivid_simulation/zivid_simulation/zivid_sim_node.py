import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import struct
import zivid
import ros2_numpy as rnp
import array
import sys
import pyarrow as pa

from sensor_msgs.msg import PointCloud2, PointField, Image
from std_srvs.srv import Trigger
from std_msgs.msg import Header
from std_srvs.srv import Trigger


class GetMockZividFrame(Node):

    def __init__(self):
        super().__init__('mock_zivid_camera')    # create a node
        self.srv = self.create_service(Trigger, 'capture', self.mock_zivid_camera_callback)  # create service server
        self.get_logger().info('MOCK /capture Service is ready.')

        # Publishers for PointCloud2 and Image
        self.pointcloud_publisher = self.create_publisher(PointCloud2, 'points/xyzrgba', 10)
        self.image_publisher = self.create_publisher(Image, '/color/image_color', 10)

        # Declare the parameter , set default -> can be passed from a launch file
        self.declare_parameter('sim_scan_path', '/home/fanuc_m16ib/ros2_ws/src/zivid_simulation/data/scan2.zdf')
        self.simScanPath = self.get_parameter('sim_scan_path').get_parameter_value().string_value
        self.caibrationMatrix = np.array([[ -9.75352466e-01, 1.67853869e-02, -2.20013127e-01, 1.98877747e+02],  
                            [6.24175072e-02, 9.77365434e-01, -2.02140734e-01, 1.36782410e+03],
                            [ 2.11640209e-01, -2.10891142e-01, -9.54323530e-01, 8.75918091e+02],
                            [ 0., 0., 0., 1. ] ], dtype=np.float32)    #Calibration JS33039 12.09.2023

    def mock_zivid_camera_callback(self, request, response):
        # Zivid initialization and loading the point cloud data
        with zivid.Application() as app:
            frame = zivid.Frame(self.simScanPath)
            point_cloud = frame.point_cloud()
<<<<<<< HEAD
            point_cloud = self.HandEyeCalibration(point_cloud)
=======
            point_cloud = self.HandEyeCalibration(point_cloud)
>>>>>>> 446dbc08d898e25719553e1a6daba449f6adeb0a

            # Extract xyz, rgb, and normals
            points = point_cloud.copy_data("xyzrgba") # structured array

            rgb = point_cloud.copy_data("bgra")[:, :, :3]  # Extract RGB channels

            # Publish PointCloud2
            #self.publish_pointcloud(points,'zivid_optical_frame')
            self.publish_pointcloud(points,'fanuc_m16ib_base_link')

            # Publish RGB image
            self.publish_image(rgb)

            # Return success response
            response.success = True
            response.message = 'Zivid point cloud and RGB image published'
        return response


    def publish_pointcloud(self, points, frame):

        # Create the PointCloud2 message
        msg = PointCloud2()
        msg.header.frame_id = frame
        msg.header.stamp = self.get_clock().now().to_msg()  
        msg.height = points.shape[0]
        msg.width = points.shape[1]
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgba', offset=12, datatype=PointField.UINT32, count=1)]

        msg.is_bigendian = False    # use little endian (bgra byte order)
        msg.is_dense = False
        msg.point_step = 16
        msg.row_step = msg.point_step * msg.width

        # convert mm (zivid) to m (ros)
        points['x'] = points['x'] / 1000
        points['y'] = points['y'] / 1000
        points['z'] = points['z'] / 1000


        msg._data = points.tobytes()    # assign to the private member _data instead of data for speed


        # Publish the message
        self.pointcloud_publisher.publish(msg)
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
            

    def HandEyeCalibration(self, point_cloud):

            pcd_base = point_cloud.transform(self.caibrationMatrix)

            return pcd_base

def main(args=None):
    rclpy.init(args=args)
    my_mock_zivid_camera = GetMockZividFrame()
    rclpy.spin(my_mock_zivid_camera)
    rclpy.shutdown()

if __name__ == '__main__':
    main()