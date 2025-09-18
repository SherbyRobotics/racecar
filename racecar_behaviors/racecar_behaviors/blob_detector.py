    #!/usr/bin/env python3

# Band-Aid to be able to use `ros2 launch`
import sys
if "/usr/local/lib/python3.12/dist-packages" in sys.path:
    sys.path.remove("/usr/local/lib/python3.12/dist-packages")

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.duration import Duration
# from rclpy.parameter_event_handler import ParameterEventHandler

import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from std_msgs.msg import String
from std_srvs.srv import Empty
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Twist, Point, TransformStamped
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
        self.color_hue = self.declare_parameter('color_hue', 125).value  # 160=purple, 100=blue, 10=Orange
        self.color_range = self.declare_parameter('color_range', 20).value
        self.color_saturation = self.declare_parameter('color_saturation', 150).value
        self.color_value = self.declare_parameter('color_value', 10).value
        self.border = self.declare_parameter('border', 1).value

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
        params.minArea = 1500
        params.maxArea = 5000000000
          
        # Set Circularity filtering parameters 
        params.filterByCircularity = True 
        params.minCircularity = 0.3
          
        # Set Convexity filtering parameters 
        params.filterByConvexity = False
        params.minConvexity = 0.2
              
        # Set inertia filtering parameters 
        params.filterByInertia = False
        params.minInertiaRatio = 0.001
        
        self.detector = cv2.SimpleBlobDetector_create(params)
        
        self.br = tf2_ros.TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        qos = QoSProfile(depth=10)
        self.image_pub = self.create_publisher(Image, 'image_detections', qos)
        self.object_pub = self.create_publisher(String, 'object_detected', qos)

        self.image_sub = message_filters.Subscriber(self, Image, 'image')
        self.depth_sub = message_filters.Subscriber(self, Image, 'depth')
        self.info_sub = message_filters.Subscriber(self, CameraInfo, 'camera_info')
        self.ts = message_filters.TimeSynchronizer([self.image_sub, self.depth_sub, self.info_sub], 10)
        self.ts.registerCallback(self.image_callback)

        self.timer = self.create_timer(1.0, self.param_callback)

        # self.handler = ParameterEventHandler(self)

        # self.hue_handle = self.handler.add_parameter_callback( parameter_name = 'color_hue', node_name='blob_detector', callback=self.hue_callback)

    def param_callback(self):
        self.color_hue = self.get_parameter('color_hue').get_parameter_value().integer_value 
        self.color_range = self.get_parameter('color_range').get_parameter_value().integer_value
        self.color_saturation = self.get_parameter('color_saturation').get_parameter_value().integer_value
        self.color_value = self.get_parameter('color_value').get_parameter_value().integer_value
        self.border = self.get_parameter('border').get_parameter_value().integer_value
        self.get_logger().info(f"param values:{self.color_hue}, {self.color_range}, {self.color_saturation}, {self.color_value}, {self.border}")
        

    # def config_callback(self, config, level):
    #     self.get_logger().info("Reconfigure Request:")

    def reset_callback(self, request, response):
        self.get_logger().info("Reset blob detector!")
        self.objects_saved = []
        return response
   
    def image_callback(self, image, depth, info):

        try:
            cv_image = self.bridge.imgmsg_to_cv2(image, "bgr8")
        except CvBridgeError as e:
             self.get_logger().info(str(e))
            
        try:
            cv_depth = self.bridge.imgmsg_to_cv2(depth, "32FC1")
        except Exception as e:
            self.get_logger().info(str(depth))
            pass
        
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        mask = cv2.inRange(hsv, np.array([self.color_hue-self.color_range,self.color_saturation,self.color_value]), np.array([self.color_hue+self.color_range,255,255]))
        keypoints = self.detector.detect(mask) 
        
        closestObject = [0,0,0] # Object pose (x,y,z) in camera frame (x->right, y->down, z->forward)
        if len(keypoints) > 0:
            cv_image = cv2.drawKeypoints(cv_image, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
            
            for i in range(0, len(keypoints)):               
                if info.k[0] > 0 and keypoints[i].pt[0] >= self.border and keypoints[i].pt[0] < cv_image.shape[1]-self.border:
                    pts_uv = np.array([[[keypoints[i].pt[0], keypoints[i].pt[1]]]], dtype=np.float32)
                    info_K = np.array(info.k).reshape([3, 3])
                    info_D = np.array(info.d)
                    info_P = np.array(info.p).reshape([3, 4])
                    pts_uv = cv2.undistortPoints(pts_uv, info_K, info_D, info_P)
                    angle = np.arcsin(-pts_uv[0][0][0]) # negative to get angle from forward x axis
                    x = pts_uv[0][0][0]
                    y = pts_uv[0][0][1]
                    # self.get_logger().info(f"({i+1}/{len(keypoints)}) {keypoints[i].pt[0]} {keypoints[i].pt[1]} -> {x} {y} angle={angle*180/np.pi} deg")
                    
                    # Get depth.
                    u = int(x * info.p[0] + info.p[2])
                    v = int(y * info.p[5] + info.p[6])
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
        if closestObject[2] > 0:
            # assuming the object is circular, use center of the object as position
            transObj = (closestObject[0], closestObject[1], closestObject[2])
            rotObj = tf_transformations.quaternion_from_euler(0, np.pi/2, -np.pi/2)
            transform = TransformStamped()
            transform.child_frame_id = self.object_frame_id
            transform.header = image.header
            transform.transform.rotation.x = rotObj[0]
            transform.transform.rotation.y = rotObj[1]
            transform.transform.rotation.z = rotObj[2]
            transform.transform.rotation.w = rotObj[3]
            transform.transform.translation.x = float(transObj[0])
            transform.transform.translation.y = float(transObj[1])
            transform.transform.translation.z = float(transObj[2])
            self.br.sendTransform(transform)  
            msg = String()
            msg.data = self.object_frame_id
            self.object_pub.publish(msg) # signal that an object has been detected
            
            # Compute object pose in map frame
            try:
                self.tf_buffer.lookup_transform(self.map_frame_id, image.header.frame_id, image.header.stamp, Duration(nanoseconds=500000000)) # 500 ms
                t = self.tf_buffer.lookup_transform(self.map_frame_id, image.header.frame_id, image.header.stamp)
                transMap = [t.transform.translation.x, t.transform.translation.y, t.transform.translation.z]
                rotMap = [t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w]
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException, tf2_ros.TransformException) as e:
                self.get_logger().info(str(e))
                return
            
            (transMap, rotMap) = multiply_transforms(transMap, rotMap, transObj, rotObj)
            
            # Compute object pose in base frame
            try:
                t = self.tf_buffer.lookup_transform(self.frame_id, image.header.frame_id, image.header.stamp, Duration(nanoseconds=500000000)) # 500 ms
                transBase = [t.transform.translation.x, t.transform.translation.y, t.transform.translation.z]
                rotBase = [t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w]
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException, tf2_ros.TransformException) as e:
                self.get_logger().info(str(e))
                return
                
            (transBase, rotBase) = multiply_transforms(transBase, rotBase, transObj, rotObj)
            distance = np.linalg.norm(transBase[0:2])
            angle = np.arcsin(transBase[1]/transBase[0]) 

            self.get_logger().info(f"Object detected at [{transMap[0]},{transMap[1]}] in {self.map_frame_id} frame! Distance and direction from robot: {distance}m {angle*180.0/np.pi}deg.")

        # debugging topic
        cv_image = cv2.bitwise_and(cv_image, cv_image, mask=mask)
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as e:
            self.get_logger().info(str(e))


def main(args=None):
    rclpy.init(args=args)
    blobDetector = BlobDetector()
    rclpy.spin(blobDetector)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
