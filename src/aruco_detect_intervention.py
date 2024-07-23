#!/usr/bin/python3
import numpy as np
import rospy
from sensor_msgs.msg import CameraInfo, Image
import cv2
from cv_bridge import CvBridge
import tf.transformations as tf
from std_msgs.msg import Float64MultiArray, MultiArrayDimension


'''
Class for aruco detection
'''
class Aruco_detect:
    def __init__(self) -> None:
        self.distortionD = []
        self.intrinsicK = []
        self.rotationR = []
        self.projectionP = []
        self.sub_camera_info = rospy.Subscriber("/turtlebot/kobuki/sensors/realsense/color/camera_info",CameraInfo, self.camera_info)
        self.sub_camera_image = rospy.Subscriber("/turtlebot/kobuki/sensors/realsense/color/image_color",Image, self.camera_image)
        self.pub_pose_obj = rospy.Publisher("/aruco/pose", Float64MultiArray, queue_size=1)
        self.marker_size_in_mm = 50
        self.objPoints = np.zeros((4, 1, 3), dtype=np.float64)
        self.objPoints[0, 0] = [-self.marker_size_in_mm / 2, self.marker_size_in_mm / 2, 0]
        self.objPoints[1, 0] = [self.marker_size_in_mm / 2, self.marker_size_in_mm / 2, 0]
        self.objPoints[2, 0] = [self.marker_size_in_mm / 2, -self.marker_size_in_mm / 2, 0]
        self.objPoints[3, 0] = [-self.marker_size_in_mm / 2, -self.marker_size_in_mm / 2, 0]
        self.rvec = np.zeros((3,1))
        self.tvec = np.zeros((3,1)) 
        self._bridge = CvBridge()
        translation = [0.136, -0.032, -0.116]
        rotation_quaternion = [0.500, 0.500, 0.500, 0.500]
        rotation_matrix = tf.quaternion_matrix(rotation_quaternion)
        translation_matrix = np.eye(4)
        translation_matrix[:3, 3] = translation
        self.transformation_matrix = np.dot(translation_matrix, rotation_matrix)
        
    ''' 
      Method getting the camera cabliration.
    '''
    def camera_info(self, info):
        self.distortionD = info.D
        # self.intrinsicK = 
        self.intrinsicK = np.float32(info.K).reshape((3,3))
        self.rotationR = info.R
        self.projectionP = info.P
    ''' 
      Method getting the object pose from aruco detection.
    '''
    def camera_image(self, img):
        cv_rgb_image = self._bridge.imgmsg_to_cv2(img, "passthrough")
        cv_rgb = cv_rgb_image.copy()
        cv_umat = cv2.UMat(cv_rgb)
        gray = cv2.cvtColor(cv_umat, cv2.COLOR_BGR2GRAY)
        aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_ARUCO_ORIGINAL)
        parameters = cv2.aruco.DetectorParameters_create()
        corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
        if corners is not None and len(corners) > 0:
            retval, self.rvec, self.tvec = cv2.solvePnP(self.objPoints, corners[0][0], self.intrinsicK, self.distortionD, self.rvec, self.tvec)
            self.tvec = np.concatenate((self.tvec, np.array([[0]])), axis=0)
            obj_pose = self.transformation_matrix @ self.tvec
            print("obj_pose",obj_pose/1000)

            # Create a Float64MultiArray message
            msg = Float64MultiArray()
            msg.layout.dim.append(MultiArrayDimension())
            msg.layout.dim[0].label = "label"
            msg.layout.dim[0].size = 4
            msg.layout.dim[0].stride = 1
            msg.layout.data_offset = 0
            msg.data = obj_pose/1000
            self.pub_pose_obj.publish(msg)
            
        else:
            print("No ArUco markers detected")

if __name__ == '__main__':
    rospy.init_node("camera_detect")
    detect_aruco = Aruco_detect()
    rospy.spin()