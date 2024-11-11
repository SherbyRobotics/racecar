#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from std_msgs.msg import String, ColorRGBA
from std_srvs.srv import Empty
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Twist, Point
import message_filters

import tf2_ros
from tf2_ros import Buffer, TransformListener
import tf_transformations
from visualization_msgs.msg import Marker
from racecar_behaviors.libbehaviors import *

class BlobDetector(Node):
    def __init__(self):
        super().__init__('blob_detector')
        self.bridge = CvBridge()

        self.map_frame_id = self.declare_parameter('map_frame_id', 'map').value
        self.frame_id = self.declare_parameter('frame_id', 'base_link').value
        self.object_frame_id = self.declare_parameter('object_frame_id', 'object').value
        self.max_distance = self.declare_parameter('max_distance', 5).value
        self.max_speed = self.declare_parameter('max_speed', 1).value
        self.max_steering = self.declare_parameter('max_steering', 0.37).value
        self.picture_distance = self.declare_parameter('picture_distance', 2).value
        self.color_hue = self.declare_parameter('color_hue', 107).value  # 160=purple, 100=blue, 10=Orange
        self.color_range = self.declare_parameter('color_range', 10).value
        self.color_saturation = self.declare_parameter('color_saturation', 50).value
        self.color_value = self.declare_parameter('color_value', 0).value
        self.border = self.declare_parameter('border', 10).value


        self.objects_saved = []
        self.wait_time = 0

        params = cv2.SimpleBlobDetector_Params()
        # Modify the parameters as needed

        params.thresholdStep = 10
        params.minThreshold = 50
        params.maxThreshold = 220
        params.minRepeatability = 2
        params.minDistBetweenBlobs = 10
        
        # Set Color filtering parameters 
        params.filterByColor = False
        params.blobColor = 255
        
        # Set Area filtering parameters 
        params.filterByArea = True
        params.minArea = 10
        params.maxArea = 5000000000
          
        # Set Circularity filtering parameters 
        params.filterByCircularity = True 
        params.minCircularity = 0.3
          
        # Set Convexity filtering parameters 
        params.filterByConvexity = False
        params.minConvexity = 0.95
              
        # Set inertia filtering parameters 
        params.filterByInertia = False
        params.minInertiaRatio = 0.1
        
        self.detector = cv2.SimpleBlobDetector_create(params)
        
        self.br = tf2_ros.TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        qos = QoSProfile(depth=10)
        self.image_pub = self.create_publisher(Image, 'image_detections', qos)
        self.object_pub = self.create_publisher(String, 'object_detected', qos)
        self.marker_pub = self.create_publisher(Marker, 'marker', qos)
        self.twist_pub = self.create_publisher(Twist, 'cmd_vel', qos)
        self.image_sub = self.create_subscription(Image, 'image', self.image_callback, 10)
        self.depth_sub = self.create_subscription(Image, 'depth', self.depth_callback, 10)
        self.info_sub = self.create_subscription(CameraInfo, 'camera_info', self.info_callback, 10)

        self.depth = Image
        self.info = CameraInfo

    def config_callback(self, config, level):
        self.get_logger().info("Reconfigure Request: {color_hue}, {color_saturation}, {color_value}, {color_range}, {border}".format(**config))
        self.color_hue = config.color_hue
        self.color_range = config.color_range
        self.color_saturation = config.color_saturation
        self.color_value = config.color_value
        self.border = config.border
        return config
        
    def reset_callback(self, request, response):
        self.get_logger().info("Reset blob detector!")
        self.objects_saved = []
        return response

    def depth_callback(self, depth_msg):
        self.depth = depth_msg
    
    def info_callback(self, info_msg):
        self.info = info_msg

    def image_callback(self, image):
        depth = self.depth
        info = self.info
        
        if self.get_clock().now().to_msg().sec - self.wait_time < 5:
            self.twist_pub.publish(Twist())
            return
        try:
            cv_image = self.bridge.imgmsg_to_cv2(image, "bgr8")
        except CvBridgeError as e:
            print(e)
            
        try:
            cv_depth = self.bridge.imgmsg_to_cv2(depth, "32FC1")
        except CvBridgeError as e:
            print(e)
        
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        mask = cv2.inRange(hsv, np.array([self.color_hue-self.color_range,self.color_saturation,self.color_value]), np.array([self.color_hue+self.color_range,255,255]))
        keypoints = self.detector.detect(mask) 
        
        closestObject = [0,0,0]
        if len(keypoints) > 0:
            cv_image = cv2.drawKeypoints(cv_image, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
            
            for i in range(0, len(keypoints)):
                if self.info.K[0] > 0 and keypoints[i].pt[0] >= self.border and keypoints[i].pt[0] < cv_image.shape[1]-self.border:
                    pts_uv = np.array([[[keypoints[i].pt[0], keypoints[i].pt[1]]]], dtype=np.float32)
                    info_K = np.array(self.info.K).reshape([3, 3])
                    info_D = np.array(self.info.D)
                    info_P = np.array(self.info.P).reshape([3, 4])
                    pts_uv = cv2.undistortPoints(pts_uv, info_K, info_D, info_P)
                    angle = np.arcsin(-pts_uv[0][0][0]) # negative to get angle from forward x axis
                    x = pts_uv[0][0][0]
                    y = pts_uv[0][0][1]
                    #rospy.loginfo("(%d/%d) %f %f -> %f %f angle=%f deg", i+1, len(keypoints), keypoints[i].pt[0], keypoints[i].pt[1], x, y, angle*180/np.pi)
                    
                    # Get depth.
                    u = int(x * self.info.P[0] + self.info.P[2])
                    v = int(y * self.info.P[5] + self.info.P[6])
                    depth = -1
                    if u >= 0 and u < cv_depth.shape[1]:
                        for j in range(0, cv_depth.shape[0]):
                            if cv_depth[j, u] > 0:
                                depth = cv_depth[j, u]
                                break
                                # is the depth contained in the blob?
                                if abs(j-v) < keypoints[i].size/2:
                                    depth = cv_depth[j, u]
                                    break
                                
                    if depth > 0 and (closestObject[2]==0 or depth<closestObject[2]):
                        closestObject[0] = x
                        closestObject[1] = y
                        closestObject[2] = depth

        # We process only the closest object detected
        if closestObject[2] > 0 and closestObject[2] < self.max_distance:
            # assuming the object is circular, use center of the object as position
            depth = closestObject[2]
            transObj = (closestObject[0]*depth, closestObject[1]*depth, depth)
            rotObj = tf_transformations.quaternion_from_euler(0, np.pi/2, -np.pi/2)
            self.br.sendTransform(transObj, rotObj,
                    image.header.stamp,
                    self.object_frame_id,
                    image.header.frame_id)  
            msg = String()
            msg.data = self.object_frame_id
            self.object_pub.publish(msg) # signal that an object has been detected
            
            # Compute object pose in map frame
            try:
                self.tf_buffer.wait_for_transform(self.map_frame_id, image.header.frame_id, image.header.stamp, rclpy.Duration(seconds=0.5))
                (transMap,rotMap) = self.tf_buffer.lookup_transform(self.map_frame_id, image.header.frame_id, image.header.stamp)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException, tf2_ros.TransformException) as e:
                self.get_logger().info(e)
                return
            
            (transMap, rotMap) = multiply_transforms((transMap, rotMap), (transObj, rotObj))
            
            objectAlreadySaved = False
            for i in range(0, len(self.objects_saved)):
                obj = self.objects_saved[i]
                if np.linalg.norm([obj[0] - transMap[0], obj[1] - transMap[1]]) < self.picture_distance:
                    objectAlreadySaved = True
                    break
            
            if not objectAlreadySaved:                
                try:
                    self.tf_buffer.wait_for_transform(self.frame_id, image.header.frame_id, image.header.stamp, rclpy.Duration(seconds=0.5))
                    (transBase,rotBase) = self.tf_buffer.lookup_transform(self.frame_id, image.header.frame_id, image.header.stamp)
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException, tf2_ros.TransformException) as e:
                    self.get_logger().info(e)
                    return
                    
                (transBase, rotBase) = multiply_transforms((transBase, rotBase), (transObj, rotObj))
                distanceToGo = np.linalg.norm(transBase[0:2])
                angle = np.arcsin(transBase[1]/transBase[0]) 
                if distanceToGo <= self.picture_distance and abs(angle) < 0.1:
                    # Take picture and save position of the object in the map
                    objId = len(self.objects_saved)
                    filename = 'object_' + str(objId) + '.jpg'
                    cv2.imwrite(filename, cv_image) 
                    self.file.write("%f %f %s\n" % (transMap[0], transMap[1], filename))
                    self.get_logger().info('Saved %s at position (%f %f, distance=%fm) in the map!', filename, transMap[0], transMap[1], distanceToGo)
                                        
                    

        # debugging topic
        if self.image_pub.get_num_connections()>0:
            cv_image = cv2.bitwise_and(cv_image, cv_image, mask=mask)
            try:
                self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
            except CvBridgeError as e:
                print(e)


def main(args=None):
    rclpy.init(args=args)
    blobDetector = BlobDetector()
    rclpy.spin(blobDetector)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
