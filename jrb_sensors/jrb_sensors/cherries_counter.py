#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
from rcl_interfaces.msg import ParameterDescriptor, ParameterType, IntegerRange, SetParametersResult


import numpy as np
import cv2
from cv_bridge import CvBridge

def nothing(x):
    pass

def opencv_to_ros(vec):
    return np.array([-vec[1], vec[0], vec[2]])

class CherriesCounter(Node):
    def __init__(self):
        super().__init__("cherries_counter")
        self.get_logger().info("init")

        self.image_publisher = self.create_publisher(Image, "debug/result_image", 2)
        self.mask_image_publisher = self.create_publisher(Image, "debug/mask_image", 2)
        self.image_subscriber = self.create_subscription(CompressedImage,"/image_raw/compressed",self.on_image_compressed,10,)
        self.camera_info_subscriber = self.create_subscription(CameraInfo, "/camera_info", self.on_camera_info, 10)

        self.cv_bridge = CvBridge()
        self.cameraMatrix = None
        self.distCoeffs = None
        self.values=np.zeros(30)

        self.declare_parameter('sat_max', 255,descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER,integer_range=[IntegerRange(from_value=0, to_value=255, step=1)],),)
        self.declare_parameter('sat_min', 81,descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER,integer_range=[IntegerRange(from_value=0, to_value=255, step=1)],),)
        self.declare_parameter('val_max', 255,descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER,integer_range=[IntegerRange(from_value=0, to_value=255, step=1)],),)
        self.declare_parameter('val_min', 10,descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER,integer_range=[IntegerRange(from_value=0, to_value=255, step=1)],),)

        self.declare_parameter('hue_low_max', 8,descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER,integer_range=[IntegerRange(from_value=0, to_value=179, step=1)],),)
        self.declare_parameter('hue_hight_min', 169,descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER,integer_range=[IntegerRange(from_value=0, to_value=179, step=1)],),)

        self.declare_parameter('min_radius', 10,descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER,integer_range=[IntegerRange(from_value=0, to_value=255, step=1)],),)
        self.declare_parameter('max_radius', 14,descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER,integer_range=[IntegerRange(from_value=0, to_value=255, step=1)],),)

        self.declare_parameter('dp', 32,descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER,integer_range=[IntegerRange(from_value=0, to_value=100, step=1)],),)
        self.declare_parameter('minDist', 19,descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER,integer_range=[IntegerRange(from_value=0, to_value=100, step=1)],),)
        self.declare_parameter('seuil_mask', 130,descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER,integer_range=[IntegerRange(from_value=0, to_value=255, step=1)],),)

        self.add_on_set_parameters_callback(self.on_update_parameters)

        # get parameters
        self.sat_max = self.get_parameter('sat_max').get_parameter_value().integer_value
        self.sat_min = self.get_parameter('sat_min').get_parameter_value().integer_value
        self.val_max = self.get_parameter('val_max').get_parameter_value().integer_value
        self.val_min = self.get_parameter('val_min').get_parameter_value().integer_value
        self.hue_low_max = self.get_parameter('hue_low_max').get_parameter_value().integer_value
        self.hue_hight_min = self.get_parameter('hue_hight_min').get_parameter_value().integer_value
        self.min_radius = self.get_parameter('min_radius').get_parameter_value().integer_value
        self.max_radius = self.get_parameter('max_radius').get_parameter_value().integer_value
        self.dp = self.get_parameter('dp').get_parameter_value().integer_value
        self.minDist = self.get_parameter('minDist').get_parameter_value().integer_value
        self.seuil_mask = self.get_parameter('seuil_mask').get_parameter_value().integer_value

        # lower boundary RED color range values; Hue (0 - 10)
        self.lower1 = np.array([0, self.sat_min, self.val_min])
        self.upper1 = np.array([self.hue_low_max, self.sat_max, self.val_max])

        # upper boundary RED color range values; Hue (160 - 180)
        self.lower2 = np.array([self.hue_hight_min,self.sat_min,self.val_min])
        self.upper2 = np.array([179,self.sat_max,self.val_max])

    def on_update_parameters(self, params):
        self.get_logger().info("Params updated")

        for param in params :
            self.get_logger().info(f"Param {param.name} updated to {param.value}")
            if param.name ==   "sat_max" :
                self.sat_max = param.value
            elif param.name == "sat_min" :
                self.sat_min = param.value
            elif param.name == "val_max" :
                self.val_max = param.value
            elif param.name == "val_min" :
                self.val_min = param.value
            elif param.name == "hue_low_max" :
                self.hue_low_max = param.value
            elif param.name == "hue_hight_min" :
                self.hue_hight_min = param.value
            elif param.name == "min_radius" :
                self.min_radius  = param.value
            elif param.name == "max_radius" :
                self.max_radius  = param.value
            elif param.name == "dp" :
                self.dp = param.value
            elif param.name == "minDist" :
                self.minDist = param.value
            elif param.name == "seuil_mask" :
                self.seuil_mask  = param.value
            else :
                self.get_logger().info("Param unknown")

        # lower boundary RED color range values; Hue (0 - 10)
        self.lower1 = np.array([0, self.sat_min, self.val_min])
        self.upper1 = np.array([self.hue_low_max, self.sat_max, self.val_max])

        # upper boundary RED color range values; Hue (160 - 180)
        self.lower2 = np.array([self.hue_hight_min,self.sat_min,self.val_min])
        self.upper2 = np.array([179,self.sat_max,self.val_max])

        return SetParametersResult(successful=True)

    def undistort_image(self, img):
        h,  w = img.shape[:2]
        newcameramtx, roi = cv2.getOptimalNewCameraMatrix(self.cameraMatrix,self.distCoeffs, (w,h), 1, (w,h))

        # undistort
        mapx, mapy = cv2.initUndistortRectifyMap(self.cameraMatrix, self.distCoeffs, None, newcameramtx, (w,h), 5)
        dst = cv2.remap(img, mapx, mapy, cv2.INTER_LINEAR)
        # crop the image
        x, y, w, h = roi
        dst = dst[y:y+h, x:x+w]
        return dst

    def on_camera_info(self, msg):
        self.get_logger().info("Got camera info, unsubscribing")

        self.cameraMatrix = np.array(msg.k).reshape(3, 3)
        self.distCoeffs = np.array(msg.d).reshape(1, 5)

        #self.get_logger().info(f"cameraMatrix: {self.cameraMatrix}")
        #self.get_logger().info(f"distCoeffs: {self.distCoeffs}")


        #self.cameraMatrix = np.array([1532.310544, 0, 473.564903 ,   0, 1521.222457 , 314.656573 , 0, 0, 1]).reshape(3, 3)
        #self.distCoeffs = np.array([-0.428682, 0.241646, 0.006020, 0.003566, 0.      ]).reshape(1,5)
        self.destroy_subscription(self.camera_info_subscriber)

    def on_image_compressed(self, msg):
        if self.cameraMatrix is None or self.distCoeffs is None:
            self.get_logger().warn("No CameraInfo yet, discard frame")
            return

        now = self.get_clock().now().to_msg()

        img_origin = self.cv_bridge.compressed_imgmsg_to_cv2(msg)
        img_origin = self.undistort_image(img_origin)

        img = cv2.GaussianBlur(img_origin, (5, 5), 0)
        hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)

        lower_mask = cv2.inRange(hsv, self.lower1, self.upper1)
        upper_mask = cv2.inRange(hsv, self.lower2, self.upper2)
        full_mask = lower_mask + upper_mask

        result = cv2.bitwise_and(img , img , mask=full_mask)
        img_GRAY = cv2.cvtColor(result, cv2.COLOR_BGR2GRAY)

        circles = cv2.HoughCircles(img_GRAY,cv2.HOUGH_GRADIENT,self.dp/10,self.minDist,
                        param1=60,param2=40,minRadius=self.min_radius,maxRadius=self.max_radius)
        if circles is not None  :
            nb=circles.shape[1]
            circles = np.uint16(np.around(circles))
            # Draw the circles
            for i in circles[0,:]:          
                roi = full_mask[i[1]-self.min_radius:i[1]+self.min_radius, i[0]-self.min_radius:i[0]+self.min_radius]
                average = roi.mean(axis=0).mean(axis=0)
                if average < self.seuil_mask :
                    cv2.circle(img,(i[0],i[1]),i[2],(255,0,0),2)
                    nb-=1
                else :
                    cv2.circle(img,(i[0],i[1]),i[2],(0,255,0),2)
                # draw the center of the circle
                cv2.circle(img,(i[0],i[1]),2,(0,0,255),3)
        else :
            nb=0
            
        self.values[-1]=nb
        self.values=np.roll(self.values,1)
        nb_balls=np.median(self.values)
        self.get_logger().info(f"Detected balls : {nb_balls}")

        #write how many balls there are
        cv2.putText(img, str(nb_balls) + " cerises", (0, 70), cv2.FONT_HERSHEY_DUPLEX,1.5, (255, 0, 0))
        
        # config circle
        cv2.circle(img,(60,100),self.max_radius,(0,0,255),2)
        cv2.circle(img,(60,100),self.min_radius,(255,0,0),2)
        cv2.circle(img,(60,140),int((self.max_radius+self.min_radius)/2),(0,255,255),2)
        cv2.circle(img,(60+self.minDist,140),int((self.max_radius+self.min_radius)/2),(0,255,255),2)      

        img_msg = self.cv_bridge.cv2_to_imgmsg(img, encoding='bgr8')
        img_msg.header.frame_id = "camera_link_optical"
        self.image_publisher.publish(img_msg)
        
        img_msg = self.cv_bridge.cv2_to_imgmsg(result, encoding='bgr8')
        img_msg.header.frame_id = "camera_link_optical"
        self.mask_image_publisher.publish(img_msg)

def main(args=None):
    rclpy.init(args=args)

    node = CherriesCounter()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node stopped cleanly")
    except Exception:
        print("Error while stopping the node")
        print(traceback.format_exc())
        raise
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
