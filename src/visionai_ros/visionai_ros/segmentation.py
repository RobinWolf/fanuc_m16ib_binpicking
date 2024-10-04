##ROS
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from visionai_ros_interfaces.srv import GetAiInference  # import the custom interface

##Python/ Vision
from detectron2.config import get_cfg
from detectron2.engine import DefaultPredictor
import numpy as np
import matplotlib.pyplot as plt
import cv2
from cv_bridge import CvBridge
import json

##local dependencies
import params
from customStructs import AI_Base
from filters import Filters
from analyze_tools import timeit_decorator



class DetectronSegmentation(Node):
    def __init__(self):
        super().__init__('visionai_segmentation')    # create a node

        # topics and services
        self.srv = self.create_service(Trigger, 'ai_inference', self.ai_inference_callback)
        self.color_subscriber = self.create_subscription(Image, '/color/image_color', self.color_subscriber_callback, 10)
        self.depth_subscriber = self.create_subscription(Image, '/depth/image_depth', self.depth_subscriber_callback, 10)
        self.mask_publisher = self.create_publisher(Image, '/color/mask', 10)
        self.bridge = CvBridge()

        self.get_logger().info("services and topics initialized")

        # load params
        self._ImagePath = params._ImagePath
        self._viz_detectron2_results = params._viz_detectron2_results
        self._class = params._class
        self._idx_forced = params._idx_forced
        self._area = params._area
        self._probability = params._probability
        self._visMask = params._visMask
        self._pickInstanceJsonPath = params._pickInstanceJsonPath

        self.get_logger().info("parameters loaded from params.py")

        cfg = get_cfg()
        cfg.merge_from_file(str(params._config_yaml_path))
        cfg.MODEL.WEIGHTS = str(params._path2Pth)
        cfg.MODEL.ROI_HEADS.SCORE_THRESH_TEST = params._confidence

        # create an instance of detectron2
        self.predictor = DefaultPredictor(cfg)

        self.get_logger().info("initialized detectron ai")



    @timeit_decorator
    def ai_inference_callback(self, request, response):
        # execute inference and set response message

        # detectron2 inference (instance segmentation)
        aiOut_struct = self.detectron2_inference(self.color_img)

        # filter detected instances for best to pick (ROI)
        aiPick_struct = self.filterPick(self.color_img, aiOut_struct)

        # transform aiPick_struct to response message and topic messages
        responsemsg = self.convertStructToMsg(aiPick_struct, response)
        return responsemsg
    

    def color_subscriber_callback(self, msg):
        # get image from topic anc convert to cv2
        try:
            # Convert ROS Image message to OpenCV image
            self.color_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Display the image
            cv2.imshow("Received Image", self.color_img)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Error converting color image: {e}")
        return

    def depth_subscriber_callback(self, msg):
        # get image from topic anc convert to cv2
        try:
            # Convert ROS Image message to OpenCV image
            self.depth_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Display the image
            cv2.imshow("Received Image", self.depth_img)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Error converting depth image: {e}")
        return
    

    def convertStructToMsg(self, aiPick_struct, response):

        # writes response message of GetAiInference service
        if aiPick_struct.score != None:
            response.success = True
            response.mask_pick = aiPick_struct.mask
            response.pick_class = aiPick_struct.pred_class
            response.pick_score = aiPick_struct.score
        else:
            response.success = False

            self.get_logger().error(f"AiInference Callback executed")

        return response


    def detectron2_inference(self, im):
        '''
        mat im: color image to pass trough the model
        
        return
        ------
        AI_Base aiOut_struct: custom struct holding contents of all instances (lists)

        '''

        # prepare rgb image
        im = cv2.cvtColor(im, cv2.COLOR_BGR2RGB)

        # do the inference on gpu
        outputs = self.predictor(im)
        len_predictions = len(outputs["instances"].to("cpu"))
        # check if nothing found in input
        if len_predictions == 0:
            self.get_logger().warn("DETECTRON: No predictions detected in image")
            return None, None
        
        # Check detectron 2 results (scores, pred_classes, pred_mask)
        # property holders for filtering module 
        aiOut_struct = AI_Base(None, None, None, None, None)
        aiOut_struct.area = []    # pixels of the predicted mask
        aiOut_struct.score = []    # probability of instance
        aiOut_struct.mask = []    # mask of instance (matrix)
        aiOut_struct.pred_class = []   # class id


        # Prediction on only class with class id  == self._class
        for i in range(len_predictions):
            #Only gather the results that are equal to self._class
            temp = outputs["instances"][i].to("cpu")
            if self._class == 100:
                aiOut_struct.score.append(temp.scores.cpu().numpy()[0])
                aiOut_struct.area.append(np.count_nonzero(temp.pred_masks.cpu().numpy().reshape(temp.pred_masks.cpu().numpy().shape[1], temp.pred_masks.cpu().numpy().shape[2])))
                aiOut_struct.mask.append(temp.pred_masks.cpu().numpy().reshape(temp.pred_masks.cpu().numpy().shape[1], temp.pred_masks.cpu().numpy().shape[2]))
                aiOut_struct.pred_class.append(temp.pred_classes.cpu().numpy()[0])
            else:
                if temp.pred_classes.cpu().numpy()[0] == self._class:
                    aiOut_struct.score.append(temp.scores.cpu().numpy()[0])
                    aiOut_struct.area.append(np.count_nonzero(temp.pred_masks.cpu().numpy().reshape(temp.pred_masks.cpu().numpy().shape[1], temp.pred_masks.cpu().numpy().shape[2])))
                    aiOut_struct.mask.append(temp.pred_masks.cpu().numpy().reshape(temp.pred_masks.cpu().numpy().shape[1], temp.pred_masks.cpu().numpy().shape[2]))
                    aiOut_struct.pred_class.append(temp.pred_classes.cpu().numpy()[0])
        
        ## OPTIONAL ##
        # visualize all instances detected one by one
        if self._viz_detectron2_results:
            for i in range(len_predictions):
                temp = outputs["instances"][i].to("cpu")
                score = temp.scores.cpu().numpy()[0]
                class_ = temp.pred_classes.cpu().numpy()[0]
                masks = temp.pred_masks.cpu().numpy()
                masks = masks.reshape(masks.shape[1], masks.shape[2])
                self.PlotOverlay(im, masks, str=str(class_)+'_'+str(score))

        return aiOut_struct
    

    def filterPick(self, im, aiOut_struct):
        '''
        AI_Base aiOut_struct: custom struct holding contents of all instances (lists)

        return
        ------
        AI_Base aiPick_struct: custom struct holding contents of the instance to pick

        '''
        
        # load and prepare rgb image
        im = cv2.cvtColor(im, cv2.COLOR_BGR2RGB)

        # init with defaults
        aiPick_struct = AI_Base(None, None, None, None, None)
        idx_pick = None
        mask_pick = None
        pick_class = None

        if self._idx_forced == None:
            # pick the instance index based on evaluation metrics
            if self._area: # Pick the one with the largest visisible area
                idx_pick = Filters.filter_area(aiOut_struct.area)

            elif self._probability: # Pick the one with the highest class probability
                idx_pick = Filters.filter_probability(aiOut_struct.score)

        else:
            # pick a manually defined instance index
            idx_pick = self._idx_forced

        # write best pick contend to AI-struct
        mask_pick = aiOut_struct.mask[idx_pick]
        pick_class = aiOut_struct.pred_class[idx_pick]
        aiPick_struct.mask = mask_pick
        aiPick_struct.pred_class = pick_class
        aiPick_struct.bbox = None
        aiPick_struct.score = aiOut_struct.score[idx_pick]
            
        ## OPTIONAL ##
        # Visualise the output mask over the image
        if self._visMask:
            self.PlotOverlay(im, mask_pick)

        return aiPick_struct
    

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
        self.get_logger().info('RGB image with mask published')



    def PlotOverlay(self, img, mask):
        mask_color = np.array([255,0,1,0.5])
        overlay_image = np.copy(img)
        overlay_image[mask] = (1 - mask_color[3]) * overlay_image[mask] + mask_color[3]*mask_color[:3]         
        self.publish_image(overlay_image)                                                                                                                      

        return