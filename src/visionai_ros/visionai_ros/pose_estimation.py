#ROS
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField
from visionai_ros_interfaces.srv import GetObjectPose, PublishCollisionGeometry  # import the custom interface

##Python/ Vision
import open3d as o3d
import numpy as np

##local dependencies
import params
from customStructs import FPFH_Base, PCD_Base
from analyze_tools import timeit_decorator

class PoseEstimation():
    def __init__(self):
        super().__init__('visionai_pose_estimation')    # create a node
        
        # topics and services
        self.collisionSrv = self.create_service(PublishCollisionGeometry, 'publish_collision_geometry', self.publish_collision_geometry_callback)
        self.srv = self.create_service(GetObjectPose, 'get_object_pose', self.get_object_pose_callback)
        self.pc_subscriber = self.create_subscription(PointCloud2, '/points/xyzbgra', self.pc_subscriber_callback, 10)
        self.pcROI_publisher = self.create_publisher(PointCloud2, '/points/xyzbgra_roi', 10)
        self.pcCollision_publisher = self.create_publisher(PointCloud2, '/points/xyzbgra_collision', 10)
        self.get_logger().info("services and topics initialized")

        # load params
        self._pathClass0 = params._pathClass0
        self._pathClass1 = params._pathClass1
        self._voxel_size_source = params._voxel_size_source
        self._voxel_size_target = params._voxel_size_target
        self._radius_feature_source = params._radius_feature_source
        self._max_nn_source = params._max_nn_source
        self._radius_feature_target = params._radius_feature_target
        self._max_nn_target = params._max_nn_target
        self._visBeforePcloud = params._visBeforePcloud
        self._distance_threshold = params._distance_threshold
        self._r_s = params._r_s
        self._r_k = params._r_k
        self._distance_threshold = params._distance_threshold
        self._ransac_iterations = params._ransac_iterations
        self._ransac_confidence = params._ransac_confidence
        self._ICPradius = params._ICPradius
        self._ICPiter_ = params._ICPiter_
        self._debug = params._debug
        self._visPcloud = params._visPcloud

        self.get_logger().info("parameters loaded from params.py")

    
    def get_object_pose_callback(self, request, response):
        # get latest pointcloud
        pcd_in = self.pcd_in_struct

        # crop ROI from input pcd
        pcd_target = self.pcd.cropPickInstance(mask_pick=request.mask, pcd_in_struct=pcd_in)

        # load registration templates from file and return in 03d format
        pickClass = request.pred_class
        pcd_source = self.getSourceTemplate(pickClass = pickClass)

        # calculate FPFH feature descriptors fro source and target pcd
        prep_source, prep_target = self.calculateFPFH(pcd_source=pcd_source, pcd_target=pcd_target)

        # feature based global registration with RANSAC
        globalTransformation = self.globalRegistration(prep_source=prep_source, prep_target=prep_target)

        # local registration with ICP
        objectTransformation = self.localRegistration(pcd_source=pcd_source, pcd_target=pcd_target, globalTransformation=globalTransformation)

        # write response message
        response = self.writeResponseMsg(response, homogenous=objectTransformation)

        return response
    

    def publish_collision_geometry_callback(self, request, response):
        # get latest pointcloud
        pcd_in = self.pcd_in_struct

        # decode request
        mask_pick = request.pick_mask

        # crop pick instance (is not a collision)
        # fatten 2D mask and get 1D list of indices where the 2D mask is false (still in rgb image with same pixel-resolution as pcd !)
        pick_idxs_pc = list(np.where(mask_pick.reshape(mask_pick.shape[0]*mask_pick.shape[1]) == False))[0]
    
        # NaN, +Inf, or -Inf values in the xyz array replaced with finite numbers and reshape to 2D array with collumns = xyz, row = n-points
        pcd_in.normals = np.nan_to_num(pcd_in.normals).reshape(-1, 3)
        pcd_in.points = np.nan_to_num(pcd_in.points).reshape(-1,3)  
        pcd_in.rgb = pcd_in.rgb.reshape(-1,3)

        # append mask point with mask idx to list if none of the corresponding point coordinates == 0 -----> maybe in structured list !!!
        pcd_Collision_struct = PCD_Base(None, None, None)
        pcd_Collision_struct.points = []
        pcd_Collision_struct.rgb = []

        for i in pick_idxs_pc:
            if pcd_in.points[i, 0] != 0 and pcd_in.points[i, 1] != 0 and pcd_in.points[i, 2] != 0:
                pcd_Collision_struct.points.append(pcd_in.points[i])
               # pcd_Collision_struct.normals.append(pcd_in.normals[i])
                pcd_Collision_struct.rgb.append(pcd_in.rgb[i] / 255 )


        self.publish_pointcloud(points,'zivid_optical_frame')

        if pick_idxs_pc != None:
            response.success = True

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

        msg._data = points.tobytes()    # assign to the private member _data instead of data for speed

        # Publish the message
        self.pcCollision_publisher.publish(msg)
        self.get_logger().info('PointCloud2 Collision Data published')
    

    def pc_subscriber_callback(self, msg):
        # get message
        pcmsg = msg

        # decode
        #TODO


        # write in custom struct
        self.pcd_in_struct = PCD_Base(None, None, None)
        self.pcd_in_struct.points = None
        self.pcd_in_struct.normals = None
        self.pcd_in_struct.rgb = None
        return
    
    def writeResponseMsg(self, response, homogenous):
        return response


    @timeit_decorator
    def cropPickInstance(self, mask_pick, pcd_in_struct):
        '''
        matrix mask_pick: mask over input image of selected instance to pick
        PCD_Base pcd_in_struct: custom struct holding input points, normals and rgb

        return
        ------
        o3d.geometry.PointCloud pcd_ROI: o3d pointcloud format of ROI

        '''

        # fatten 2D mask and get 1D list of indices where the 2D mask is true (still in rgb image with same pixel-resolution as pcd !)
        pick_idxs_pc = list(np.where(mask_pick.reshape(mask_pick.shape[0]*mask_pick.shape[1]) == True))[0]
    
        # NaN, +Inf, or -Inf values in the xyz array replaced with finite numbers and reshape to 2D array with collumns = xyz, row = n-points
        pcd_in_struct.normals = np.nan_to_num(pcd_in_struct.normals).reshape(-1, 3)
        pcd_in_struct.points = np.nan_to_num(pcd_in_struct.points).reshape(-1,3)  
        pcd_in_struct.rgb = pcd_in_struct.rgb.reshape(-1,3)

        # append mask point with mask idx to list if none of the corresponding point coordinates == 0
        pcd_ROI_struct = PCD_Base(None, None, None)
        pcd_ROI_struct.points = []
        pcd_ROI_struct.normals = []
        pcd_ROI_struct.rgb = []

        for i in pick_idxs_pc:
            if pcd_in_struct.points[i, 0] != 0 and pcd_in_struct.points[i, 1] != 0 and pcd_in_struct.points[i, 2] != 0:
                pcd_ROI_struct.points.append(pcd_in_struct.points[i])
                pcd_ROI_struct.normals.append(pcd_in_struct.normals[i])
                pcd_ROI_struct.rgb.append(pcd_in_struct.rgb[i] / 255 )

        # transform points in lists to o3d format
        pcd_ROI = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(pcd_ROI_struct.points))
        pcd_ROI.normals = o3d.utility.Vector3dVector(pcd_ROI_struct.normals)
        pcd_ROI.colors = o3d.utility.Vector3dVector( pcd_ROI_struct.rgb)

        ## OPTIONAL ##
        if self._saveCroppedPly:
            self.exportROIPLY(pcd_ROI)

        if self._visPcloud:
            #TODO
            pass


        return pcd_ROI


    @timeit_decorator
    def getSourceTemplate(self, pickClass):
        '''
        int pickClass: class to pick (currently top/ bottom welding nuts)

        return
        ------
        o3d.geometry.PointCloud pcd_source: template source pcd with local reference set to register into ROI

        '''

        # select source template path (currently top/ bottom templates)
        if pickClass == 0:
            source_path = self._pathClass0
        elif pickClass == 1:
            source_path = self._pathClass1
                
        # read template/ source from file
        pcd_source = o3d.io.read_point_cloud(str(source_path))

        return pcd_source
    

    @timeit_decorator
    def calculateFPFH(self, pcd_source, pcd_target):
        '''
        o3d.geometry.PointCloud pcd_source: template source pcd
        o3d.geometry.PointCloud pcd_target: cropped ROI target pcd

        return
        ------
        FPFH_Base prep_source: custom struct to hold downsampled pcds and calculated fpfh
        FPFH_Base prep_target: custom struct to hold downsampled pcds and calculated fpfh

        '''
        
        prep_source = FPFH_Base(None, None)
        prep_target = FPFH_Base(None, None)

        # downsample pcd to specific voxel size (target and source have to be the same !!!)
        prep_source.pcd_downsampled = pcd_source.voxel_down_sample(self._voxel_size_source)
        prep_target.pcd_downsampled = pcd_target.voxel_down_sample(self._voxel_size_target)
 
        # compute fpfh feature descriptors of each point from the input pcd in respect to neighbour points (describing local geometry)
        prep_source.fpfh = o3d.pipelines.registration.compute_fpfh_feature(prep_source.pcd_downsampled,
            o3d.geometry.KDTreeSearchParamHybrid(radius=self._radius_feature_source, max_nn=self._max_nn_source))
        prep_target.fpfh = o3d.pipelines.registration.compute_fpfh_feature(prep_target.pcd_downsampled,
            o3d.geometry.KDTreeSearchParamHybrid(radius=self._radius_feature_target, max_nn=self._max_nn_target))
        
        ## OPTIONAL ##
        if self._visBeforePcloud:
            #TODO
            pass

        return prep_source, prep_target
    

    @timeit_decorator
    def globalRegistration(self, prep_source, prep_target):
        '''
        FPFH_Base prep_source: custom struct to hold downsampled pcds and calculated fpfh
        FPFH_Base prep_target: custom struct to hold downsampled pcds and calculated fpfh

        return
        ------
        matrix globalTransformation: homogenous transformation to apply to source pcd after feature based global registration

        '''

        # perform ransac registration based on precomputed fpfh feature descriptors
        result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
            prep_source.pcd_downsampled, prep_target.pcd_downsampled, prep_source.fpfh, prep_target.fpfh, True,   # input clouds and feautre descriptors
            self._distance_threshold,
            o3d.pipelines.registration.TransformationEstimationPointToPoint(False), # use point to point estimation method without considering normals (False)
            self._r_k,
            [   # check validity of correspondance points (to choose best transformation over all iterations)
                o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(self._r_s),
                o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(self._distance_threshold)
            ],
            o3d.pipelines.registration.RANSACConvergenceCriteria(self._ransac_iterations, self._ransac_confidence)) # specify when ransac algorithm should converge
        
        globalTransformation = result.transformation

        ## OPTIONAL: ##
        if self._debug:
            print(f'RANSAC result: {result}')
            print(f'Transformation of Instance to Pick after global Registration : {globalTransformation}')
        
        return globalTransformation
        

    @timeit_decorator
    def localRegistration(self, pcd_source, pcd_target, globalTransformation):
        '''
        o3d.geometry.PointCloud pcd_source: template source pcd
        o3d.geometry.PointCloud pcd_target: cropped ROI target pcd
        matrix globalTransformation: homogenous transformation to apply to source pcd after feature based global registration

        return
        ------
        matrix globalANDlocalTransformation: homogenous transformation to apply to source pcd after loal registration

        '''
        
        result_icp = o3d.pipelines.registration.registration_icp(
                    pcd_source, pcd_target, self._ICPradius, globalTransformation,
                    #o3d.pipelines.registration.TransformationEstimationPointToPlane(),
                    o3d.pipelines.registration.TransformationEstimationPointToPoint(),
                    o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=self._ICPiter_))
        
        globalANDlocalTransformation = result_icp.transformation

        ## OPTIONAL: ##
        if self._debug:
            print(f'Transformation of Instance to Pick in Robot Base Frame : {globalANDlocalTransformation}')

        if self._visPcloud:
            #self.interactiveVis(pcd_source, pcd_target,  globalTransformation, "RANSAC global Registration")
            #self.interactiveVis(pcd_source, pcd_target,  globalANDlocalTransformation, "ICP local Registration")
            self.headlessVis(pcd_source, pcd_target,  globalTransformation, "RANSAC_global_Registration.png")
            self.headlessVis(pcd_source, pcd_target,  globalANDlocalTransformation, "ICP_local_Registration.png")

        return globalANDlocalTransformation
    
    @timeit_decorator
    def exportCollisionPLY(self, mask_pick, pcd_in_struct):
        '''
        matrix mask_pick: mask over input image of selected instance to pick
        PCD_Base pcd_in_struct: custom struct holding input points, normals and rgb

        return
        ------
        None

        '''

        # fatten 2D mask and get 1D list of indices where the 2D mask is true (still in rgb image with same pixel-resolution as pcd !)
        pick_idxs_pc = list(np.where(mask_pick.reshape(mask_pick.shape[0]*mask_pick.shape[1]) == True))[0]

        # transform points in lists to o3d format
        point_cloud = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(pcd_in_struct.points.reshape(-1, 3)))
        point_cloud.normals = o3d.utility.Vector3dVector(pcd_in_struct.normals.reshape(-1,3))
        point_cloud = point_cloud.remove_non_finite_points()

        # delet cropped points (mask) from pcd
        point_cloud = point_cloud.select_by_index(pick_idxs_pc, invert=True)

        # downsample pointcoud
        #point_cloud = point_cloud.voxel_down_sample(voxel_size=self._downsample_voxel_size_full_scan)

        return None
    
    @timeit_decorator
    def exportROIPLY(self, pcd):
        '''
        o3d.geometry.PointCloud pcd_ROI: pcd to export

        return
        ------
        None

        '''
        
        if self._debug:
            print(f"saving cropped PLY to {str(self._croppedPLYPath)}")

        o3d.io.write_point_cloud(str(self._croppedPLYPath), pcd)

        return None


    
    
