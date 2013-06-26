#!/usr/bin/env python

PKG  = 'velma_calibration'
NODE = 'collect_data_node'
import roslib; roslib.load_manifest(PKG)
import rospy

import message_filters
from sensor_msgs.msg import *
#from calibration_msgs.msg import *
from cv_bridge import CvBridge, CvBridgeError

from velma_calibration.srv import *
#from cob_camera_calibration import Checkerboard, CheckerboardDetector, AsymmetricCirclegrid, ACirclegridDetector, cv2util

from calibration_object import *
from calibration_object_detector import *

import cv2util

import tf
import numpy as np
import scipy.io as sio

CHECKERBOARD_PATTERN_SIZE = (3,7)
CHECKERBOARD_SQUARE_SIZE = 0.02
CHECKERBOARD_NAME = "cb_9x6"
CHECKERBOARD_CHAIN = "arm_chain"

class DataCollector():
    '''
    @summary: Collects data for robot calibration.
    
    Subscribes to various topics needed (e.g. images, camera infos, joint angles) and 
    provides a service. When service is called, a set of samples is recorded, 
    processed (e.g. checkerboards are detected) and combined to a RobotMeasurement message 
    which is published as /robot_measurement.
    '''
    
    def __init__(self):
        '''
        Set up subscribers, publishers and local storage
        ''' 
        rospy.init_node(NODE)
        print "==> %s started " % NODE
        
        # get joint names for arm
        #if rospy.has_param("arm_controller/joint_names"): # real hardware
        #    self.arm_joint_names = rospy.get_param("arm_controller/joint_names")
        #elif rospy.has_param("arm_controller/joints"): # simulation
        #    self.arm_joint_names = rospy.get_param("arm_controller/joints")
        #else: 
        #    print "Could not get joint names for arm from parameter server. exiting..."
        #    exit(-1)
            
        # get joint names for head
        if rospy.has_param("head_controller/joint_names"): # real hardware
            self.head_joint_names = rospy.get_param("head_controller/joint_names")
        else: 
            print "Could not get joint names for head from parameter server. exiting..."
            exit(-1)
        # get joint names for right_arm
        if rospy.has_param("right_arm_controller/joint_names"): # real hardware
            self.right_arm_joint_names = rospy.get_param("right_arm_controller/joint_names")
        else: 
            print "Could not get joint names for right arm from parameter server. exiting..."
            self.right_arm_joint_names = []
        
        if rospy.has_param('~calib_file_path'):
          self.calib_file_path = rospy.get_param('~calib_file_path')
        else:
          self.calib_file_path = '/tmp/'
        
        # CvBridge
        self.bridge = CvBridge() 
        
        # initialize private storage
        self._arm_joint_msg_received = False
        self._arm_joint_msg = None
        self._head_joint_msg_received = False
        self._head_joint_msg = None
        self._left = {}
        self._left_received = False
        self._right = {}
        self._right_received = False
        self._kinect_left_rgb = {}
        self._kinect_left_rgb_received = False
        self._kinect_right_rgb = {}
        self._kinect_right_rgb_received = False
        self.counter = 1
        
        
        self.p_img_l = []
        self.p_img_r = []
        self.p_img_kl = []
        self.p_img_kr = []
        
        self.p_world_l = []
        self.p_world_r = []
        self.p_world_kl = []
        self.p_world_kr = []
        
        self.head_joints_l = []
        self.head_joints_r = []
        self.head_joints_kl = []
        self.head_joints_kr = []
        
        self.right_arm_joints_l = []
        self.right_arm_joints_r = []
        self.right_arm_joints_kl = []
        self.right_arm_joints_kr = []
        
        self.right_arm_joints_trq_l = []
        self.right_arm_joints_trq_r = []
        self.right_arm_joints_trq_kl = []
        self.right_arm_joints_trq_kr = []
        
        #  init publisher / subscriber
        self._image_pub_left        = rospy.Publisher("/robot_measurement_image_left",  Image) #DEBUG
        self._image_pub_right       = rospy.Publisher("/robot_measurement_image_right", Image) #DEBUG
        self._image_pub_kinect_left_rgb  = rospy.Publisher("/robot_measurement_image_kinect_left_rgb", Image) #DEBUG
        self._image_pub_kinect_right_rgb  = rospy.Publisher("/robot_measurement_image_kinect_right_rgb", Image) #DEBUG
        
        self._sub_joint_states      = rospy.Subscriber( "/joint_states", JointState, self._callback_joints)
        
        # left camera
        self._sub_left_info         = message_filters.Subscriber("/stereo_left/camera_info", CameraInfo)
        self._sub_left_image_rect   = message_filters.Subscriber("/stereo_left/image_rect_color", Image)
        self._sub_left              = message_filters.TimeSynchronizer([self._sub_left_info, 
                                                                        self._sub_left_image_rect], 15)
        self._sub_left.registerCallback(self._callback_left)
        
        # right camera
        #self._sub_right_info         = message_filters.Subscriber("/stereo/right/camera_info", CameraInfo)  
        #self._sub_right_image_color  = message_filters.Subscriber("/stereo/right/image_raw", Image)
        #self._sub_right_image_rect   = message_filters.Subscriber("/stereo/right/image_raw", Image)
        #self._sub_right              = message_filters.TimeSynchronizer([self._sub_right_info, 
        #                                                                self._sub_right_image_color, 
        #                                                                self._sub_right_image_rect], 15)
        #self._sub_right.registerCallback(self._callback_right)
        
        # kinect rgb left
        self._sub_kinect_left_rgb_info         = message_filters.Subscriber("/left_kinect/rgb/camera_info", CameraInfo)
        self._sub_kinect_left_rgb_image_color  = message_filters.Subscriber("/left_kinect/rgb/image_rect_color", Image)
        self._sub_kinect_left_rgb              = message_filters.TimeSynchronizer([self._sub_kinect_left_rgb_info, 
                                                                        self._sub_kinect_left_rgb_image_color], 15)
        self._sub_kinect_left_rgb.registerCallback(self._callback_kinect_left_rgb)
        
        # kinect rgb right
        self._sub_kinect_right_rgb_info         = message_filters.Subscriber("/right_kinect/rgb/camera_info", CameraInfo)  
        self._sub_kinect_right_rgb_image_color  = message_filters.Subscriber("/right_kinect/rgb/image_rect_color", Image)
        self._sub_kinect_right_rgb              = message_filters.TimeSynchronizer([self._sub_kinect_right_rgb_info, 
                                                                        self._sub_kinect_right_rgb_image_color], 15)
        self._sub_kinect_right_rgb.registerCallback(self._callback_kinect_right_rgb)
        
        print "==> done with initialization"

    def _callback_left(self, camera_info, image_rect):
        '''
        Callback function for left camera message filter
        '''
        #print "DEBUG: callback left"
        self._left["camera_info"] = camera_info
        self._left["image_rect"] = image_rect
#        if self._left_received == False:
#            print "--> left sample received (this only prints once!)"
        self._left_received = True
        
    def _callback_right(self, camera_info, image_rect):
        '''
        Callback function for right camera message filter
        '''
        #print "DEBUG: callback right"
        self._right["camera_info"] = camera_info
        self._right["image_rect"] = image_rect
#        if self._right_received == False:
#            print "--> right sample received (this only prints once!)"
        self._right_received = True

    def _callback_kinect_left_rgb(self, camera_info, image_color):
        '''
        Callback function for kinect rgb message filter
        '''
        #print "DEBUG: callback kinect_rgb"
        self._kinect_left_rgb["camera_info"] = camera_info
        self._kinect_left_rgb["image_color"] = image_color
#        if self._kinect_rgb_received == False:
#            print "--> kinect sample received (this only prints once!)"
        self._kinect_left_rgb_received = True
        
    def _callback_kinect_right_rgb(self, camera_info, image_color):
        '''
        Callback function for kinect rgb message filter
        '''
        #print "DEBUG: callback kinect_rgb"
        self._kinect_right_rgb["camera_info"] = camera_info
        self._kinect_right_rgb["image_color"] = image_color
#        if self._kinect_rgb_received == False:
#            print "--> kinect sample received (this only prints once!)"
        self._kinect_right_rgb_received = True
        
    def _callback_joints(self, msg):
        '''
        Callback function for joint angles messages
        '''
        # head
        if self.head_joint_names[0] in msg.name:
          pos = []
          header = msg.header
          for name in self.head_joint_names:
            pos.append(msg.position[msg.name.index(name)])
          
          # create JointState message
          joint_msg = JointState()
          joint_msg.header = msg.header
          joint_msg.name = self.head_joint_names
          joint_msg.position = pos
          
          # safe joint state msg
          self._head_joint_msg = joint_msg
          self._head_joint_msg_received = True
        # right arm
        if (len(self.right_arm_joint_names) > 0):
          if self.right_arm_joint_names[0] in msg.name:
            pos = []
            trq = []
            header = msg.header
            for name in self.right_arm_joint_names:
              pos.append(msg.position[msg.name.index(name)])
              trq.append(msg.effort[msg.name.index(name)])
            
            # create JointState message
            joint_msg = JointState()
            joint_msg.header = msg.header
            joint_msg.name = self.right_arm_joint_names
            joint_msg.position = pos
            joint_msg.effort = trq
            
            # safe joint state msg
            self._right_arm_joint_msg = joint_msg
            self._right_arm_joint_msg_received = True
    def run(self):
        '''
        Main method, starts service to provide capture functionality
        '''
        rospy.sleep(1)
        
        self.listener = tf.TransformListener()
        
        rospy.sleep(2)
        
        [T_l, R_l] = self.listener.lookupTransform('/stereo_left_optical_frame', '/head_tilt_link', rospy.Time(0))
        [T_kl, R_kl] = self.listener.lookupTransform('/kinect_left_rgb_optical_frame', '/head_tilt_link', rospy.Time(0))
        [T_kr, R_kr] = self.listener.lookupTransform('/kinect_right_rgb_optical_frame', '/head_tilt_link', rospy.Time(0))
        [T_ah, R_ah] = self.listener.lookupTransform('/calib_right_arm_base_link', '/head_frame', rospy.Time(0))
        # Start service
        srv = rospy.Service('/collect_data/capture', Capture, self._collect)
        rospy.loginfo("service '/collect_data/capture' started, waiting for requests...")
        
        rospy.spin()
        
        # left
        p_img_l = np.empty((len(self.p_img_l),), dtype=np.object)
        for i in range(len(self.p_img_l)):
          p_img_l[i] = self.p_img_l[i]
        
        p_world_l = np.empty((len(self.p_world_l),), dtype=np.object)
        for i in range(len(self.p_world_l)):
          p_world_l[i] = self.p_world_l[i]
        
        head_joints_l = np.empty((len(self.head_joints_l),), dtype=np.object)
        for i in range(len(self.head_joints_l)):
          head_joints_l[i] = self.head_joints_l[i]
        
        right_arm_joints_l = np.empty((len(self.right_arm_joints_l),), dtype=np.object)
        for i in range(len(self.right_arm_joints_l)):
          right_arm_joints_l[i] = self.right_arm_joints_l[i]
        
        right_arm_joints_trq_l = np.empty((len(self.right_arm_joints_trq_l),), dtype=np.object)
        for i in range(len(self.right_arm_joints_trq_l)):
          right_arm_joints_trq_l[i] = self.right_arm_joints_trq_l[i]
          
        # right
        p_img_r = np.empty((len(self.p_img_r),), dtype=np.object)
        for i in range(len(self.p_img_r)):
          p_img_r[i] = self.p_img_r[i]
        
        p_world_r = np.empty((len(self.p_world_r),), dtype=np.object)
        for i in range(len(self.p_world_r)):
          p_world_r[i] = self.p_world_r[i]
        
        head_joints_r = np.empty((len(self.head_joints_r),), dtype=np.object)
        for i in range(len(self.head_joints_r)):
          head_joints_r[i] = self.head_joints_r[i]
        
        right_arm_joints_r = np.empty((len(self.right_arm_joints_r),), dtype=np.object)
        for i in range(len(self.right_arm_joints_r)):
          right_arm_joints_r[i] = self.right_arm_joints_r[i]
        
        right_arm_joints_trq_r = np.empty((len(self.right_arm_joints_trq_r),), dtype=np.object)
        for i in range(len(self.right_arm_joints_trq_r)):
          right_arm_joints_trq_r[i] = self.right_arm_joints_trq_r[i]
        
        # kinect left
        p_img_kl = np.empty((len(self.p_img_kl),), dtype=np.object)
        for i in range(len(self.p_img_kl)):
          p_img_kl[i] = self.p_img_kl[i]
        
        p_world_kl = np.empty((len(self.p_world_kl),), dtype=np.object)
        for i in range(len(self.p_world_kl)):
          p_world_kl[i] = self.p_world_kl[i]
        
        head_joints_kl = np.empty((len(self.head_joints_kl),), dtype=np.object)
        for i in range(len(self.head_joints_kl)):
          head_joints_kl[i] = self.head_joints_kl[i]
        
        right_arm_joints_kl = np.empty((len(self.right_arm_joints_kl),), dtype=np.object)
        for i in range(len(self.right_arm_joints_kl)):
          right_arm_joints_kl[i] = self.right_arm_joints_kl[i]
        
        right_arm_joints_trq_kl = np.empty((len(self.right_arm_joints_trq_kl),), dtype=np.object)
        for i in range(len(self.right_arm_joints_trq_kl)):
          right_arm_joints_trq_kl[i] = self.right_arm_joints_trq_kl[i]
        
        # kinect right
        p_img_kr = np.empty((len(self.p_img_kr),), dtype=np.object)
        for i in range(len(self.p_img_kr)):
          p_img_kr[i] = self.p_img_kr[i]
        
        p_world_kr = np.empty((len(self.p_world_kr),), dtype=np.object)
        for i in range(len(self.p_world_kr)):
          p_world_kr[i] = self.p_world_kr[i]
        
        head_joints_kr = np.empty((len(self.head_joints_kr),), dtype=np.object)
        for i in range(len(self.head_joints_kr)):
          head_joints_kr[i] = self.head_joints_kr[i]
        
        right_arm_joints_kr = np.empty((len(self.right_arm_joints_kr),), dtype=np.object)
        for i in range(len(self.right_arm_joints_kr)):
          right_arm_joints_kr[i] = self.right_arm_joints_kr[i]
        
        right_arm_joints_trq_kr = np.empty((len(self.right_arm_joints_trq_kr),), dtype=np.object)
        for i in range(len(self.right_arm_joints_trq_kr)):
          right_arm_joints_trq_kr[i] = self.right_arm_joints_trq_kr[i]
        
        sio.savemat(self.calib_file_path + 'calibration_data.mat', { "p_img_l":p_img_l, "p_world_l":p_world_l, "head_joints_l":head_joints_l, "right_arm_joints_l":right_arm_joints_l,"right_arm_joints_trq_l":right_arm_joints_trq_l, "num_images_left":float(len(self.p_img_l)), "p_img_r":p_img_r, "p_world_r":p_world_r, "head_joints_r":head_joints_r, "right_arm_joints_r":right_arm_joints_r, "right_arm_joints_trq_r":right_arm_joints_trq_r, "num_images_right":float(len(self.p_img_r)), "p_img_kl":p_img_kl, "p_world_kl":p_world_kl, "head_joints_kl":head_joints_kl, "right_arm_joints_kl":right_arm_joints_kl, "right_arm_joints_trq_kl":right_arm_joints_trq_kl, "num_images_kinect_left":float(len(self.p_img_kl)), "p_img_kr":p_img_kr, "p_world_kr":p_world_kr, "head_joints_kr":head_joints_kr, "right_arm_joints_kr":right_arm_joints_kr, "right_arm_joints_trq_kr":right_arm_joints_trq_kr, "num_images_kinect_right":float(len(self.p_img_kr)), "T_l":T_l, "R_l":R_l, "T_kl":T_kl, "R_kl":R_kl, "T_kr":T_kr, "R_kr":R_kr, "T_ah":T_ah, "R_ah":R_ah, "i_l":self._left["camera_info"].P, "i_kr":self._kinect_right_rgb["camera_info"].P, "i_kl":self._kinect_left_rgb["camera_info"].P})

    def _collect(self, data):
        '''
        Executed on service call. Logs and calls _capture_and_pub
        '''
        rospy.loginfo("capturing sample %.2i"%self.counter)
        res = self._capture_and_pub("sample%.2i"%self.counter, CHECKERBOARD_NAME,
                                                               CHECKERBOARD_CHAIN,
                                                               CHECKERBOARD_PATTERN_SIZE, 
                                                               CHECKERBOARD_SQUARE_SIZE)
        self.counter += 1
        return CaptureResponse(res)

    def _capture_and_pub(self, sample_id, target_id, chain_id, pattern_size, square_size):
        '''
        Main capturing function. Gets a set of recent messages for all needed topics.
        Processes messages and creates RobotMeasuerment message which is published.
        
        @param sample_id: Sample identifier (e.g. sample01)
        @type  sample_id: string
        
        @param target_id: Name of checkerboard (e.g. cb_9x6)
        @type  target_id: string
        
        @param chain_id: Name of dh chain to which checkerboard is attached (e.g. arm_chain)
        @type  chain_id: string
        
        @param pattern_size: Size of checkerboard pattern as defined by opencv (e.g. (9, 6))
        @type  pattern_size: tuple(x, y)
        
        @param square_size: Size of checkerboard sqaures (im m)
        @type  square_size: float
        '''
        # capture measurements
        # --------------------
        self._left_received = False
        self._right_received = False
        self._kinect_left_rgb_received = False
        self._kinect_right_rgb_received = False
        start_time = rospy.Time.now()
        while (not self._left_received or not self._kinect_left_rgb_received or not self._kinect_right_rgb_received or not self._head_joint_msg_received or not (self._right_arm_joint_msg_received or (len(self.right_arm_joint_names) == 0))):
            rospy.sleep(0.005)
            # print warning every 2 seconds if one of the messages is still missing...
            if start_time + rospy.Duration(2.0) < rospy.Time.now():
                if not self._left_received: print "--> still waiting for sample from left"
                if not self._right_received: print "--> still waiting for sample from right"
                if not self._kinect_left_rgb_received: print "--> still waiting for sample from left kinect"
                if not self._kinect_right_rgb_received: print "--> still waiting for sample from right kinect"
                if not self._head_joint_msg_received: print "--> still waiting for sample from head joints"
                start_time = rospy.Time.now()
        latest_left = self._left
        latest_right = self._right
        latest_kinect_left_rgb = self._kinect_left_rgb
        latest_kinect_right_rgb = self._kinect_right_rgb
        latest_head_joint_msg = self._head_joint_msg
        latest_right_arm_joint_msg = self._right_arm_joint_msg
        
        # set up checkerboard and checkerboard detector
        # ---------------------------------------------
        #checkerboard = Checkerboard(pattern_size, square_size)
        #checkerboard_detector = CheckerboardDetector(checkerboard)
        
        checkerboard = AsymmetricCirclegrid(pattern_size, square_size)
        checkerboard_detector = ACirclegridDetector(checkerboard)
        
        # detect cb left
        # --------------
        cvImage = self.bridge.imgmsg_to_cv(latest_left["image_rect"], "mono8")
        image = cv2util.cvmat2np(cvImage)
        
        corners_left = checkerboard_detector.detect_image_points(image, is_grayscale=True)
        if corners_left != None:
            print "cb found: left"
        else:
            print "cb not found: left"
        
        # detect cb right
        # --------------
        #cvImage = self.bridge.imgmsg_to_cv(latest_right["image_rect"], "mono8")
        #image = cv2util.cvmat2np(cvImage)
        
        #corners_right = checkerboard_detector.detect_image_points(image, is_grayscale=True)
        #if corners_right != None:
        #  print "cb found: right"
        #else:
        #  print "cb not found: right"
        
        # detect cb kinect_left_rgb
        # --------------------
        cvImage = self.bridge.imgmsg_to_cv(latest_kinect_left_rgb["image_color"], "mono8")
        image = cv2util.cvmat2np(cvImage)
        
        corners_kinect_left = checkerboard_detector.detect_image_points(image, is_grayscale=True)
        if corners_kinect_left != None:
            print "cb found: kinect_left_rgb"
        else:
            print "cb not found: kinect_left_rgb"
        
        # detect cb kinect_right_rgb
        # --------------------
        cvImage = self.bridge.imgmsg_to_cv(latest_kinect_right_rgb["image_color"], "mono8")
        image = cv2util.cvmat2np(cvImage)
        
        corners_kinect_right = checkerboard_detector.detect_image_points(image, is_grayscale=True)
        if corners_kinect_right != None:
            print "cb found: kinect_right_rgb"
        else:
            print "cb not found: kinect_right_rgb"
        
        # DEBUG publish pic
        # -----------------
        #self._image_pub_left.publish(latest_left["image_color"])
        #self._image_pub_right.publish(latest_right["image_color"])
        #self._image_pub_kinect_left_rgb.publish(latest_kinect_left_rgb["image_color"])
        #self._image_pub_kinect_right_rgb.publish(latest_kinect_right_rgb["image_color"])
        
        if(corners_kinect_left != None):
          image_points_kl = np.empty([2, pattern_size[0] * pattern_size[1]])
          for i in range(len(corners_kinect_left)) :
            image_points_kl[0, i] = corners_kinect_left[i][0][0]
            image_points_kl[1, i] = corners_kinect_left[i][0][1]
          self.p_img_kl.append(image_points_kl)
          
          pattern = checkerboard.get_pattern_points()
          
          pattern_points = np.empty([3, pattern_size[0] * pattern_size[1]])
          
          for i in range(len(pattern)) :
            pattern_points[0, i] = pattern[i][0]
            pattern_points[1, i] = pattern[i][1]
            pattern_points[2, i] = 0
          
          self.p_world_kl.append(pattern_points)
          self.head_joints_kl.append(latest_head_joint_msg.position)
          if(len(self.right_arm_joint_names) > 0):
            self.right_arm_joints_kl.append(latest_right_arm_joint_msg.position)
            self.right_arm_joints_trq_kl.append(latest_right_arm_joint_msg.effort)
        
        if((corners_left != None)):
          image_points_l = np.empty([2, pattern_size[0] * pattern_size[1]])
          for i in range(len(corners_left)) :
            image_points_l[0, i] = corners_left[i][0][0]
            image_points_l[1, i] = corners_left[i][0][1]
          self.p_img_l.append(image_points_l)
          
          pattern = checkerboard.get_pattern_points()
          
          pattern_points = np.empty([3, pattern_size[0] * pattern_size[1]])
          
          for i in range(len(pattern)) :
            pattern_points[0, i] = pattern[i][0]
            pattern_points[1, i] = pattern[i][1]
            pattern_points[2, i] = 0
          
          self.p_world_l.append(pattern_points)
          self.head_joints_l.append(latest_head_joint_msg.position)
          if(len(self.right_arm_joint_names) > 0):
            self.right_arm_joints_l.append(latest_right_arm_joint_msg.position)
            self.right_arm_joints_trq_l.append(latest_right_arm_joint_msg.effort)
        
        if((corners_kinect_right != None)):
          image_points_kr = np.empty([2, pattern_size[0] * pattern_size[1]])
          for i in range(len(corners_kinect_right)) :
            image_points_kr[0, i] = corners_kinect_right[i][0][0]
            image_points_kr[1, i] = corners_kinect_right[i][0][1]
          self.p_img_kr.append(image_points_kr)
          
          pattern = checkerboard.get_pattern_points()
          
          pattern_points = np.empty([3, pattern_size[0] * pattern_size[1]])
          
          for i in range(len(pattern)) :
            pattern_points[0, i] = pattern[i][0]
            pattern_points[1, i] = pattern[i][1]
            pattern_points[2, i] = 0
          
          self.p_world_kr.append(pattern_points)
          self.head_joints_kr.append(latest_head_joint_msg.position)
          if(len(self.right_arm_joint_names) > 0):
            self.right_arm_joints_kr.append(latest_right_arm_joint_msg.position)
            self.right_arm_joints_trq_kr.append(latest_right_arm_joint_msg.effort)
        
        return True


if __name__ == "__main__":
    collector = DataCollector()
    collector.run()

