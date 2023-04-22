#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
from rcl_interfaces.msg import ParameterDescriptor, ParameterType, IntegerRange


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


    def on_update_parameters(self, params):
        self.get_logger().info("Params updated")

        return SetParametersResult(successful=True)

    def on_camera_info(self, msg):
        self.get_logger().info("Got camera info, unsubscribing")
        #self.cameraMatrix = np.array(msg.k).reshape(3, 3)
        #self.distCoeffs = np.array(msg.d).reshape(1, 5)
        self.destroy_subscription(self.camera_info_subscriber)

    def on_image_compressed(self, msg):
        #self.get_logger().info("in on_image_compressed")
        #if self.cameraMatrix is None or self.distCoeffs is None:
        #    self.get_logger().warn("No CameraInfo yet, discard frame")
        #    return

        now = self.get_clock().now().to_msg()

        img_origin = self.cv_bridge.compressed_imgmsg_to_cv2(msg)

        # get parameters
        sat_max = self.get_parameter('sat_max').get_parameter_value().integer_value
        sat_min = self.get_parameter('sat_min').get_parameter_value().integer_value
        val_max = self.get_parameter('val_max').get_parameter_value().integer_value
        val_min = self.get_parameter('val_min').get_parameter_value().integer_value
        hue_low_max = self.get_parameter('hue_low_max').get_parameter_value().integer_value
        hue_hight_min = self.get_parameter('hue_hight_min').get_parameter_value().integer_value
        min_radius = self.get_parameter('min_radius').get_parameter_value().integer_value
        max_radius = self.get_parameter('max_radius').get_parameter_value().integer_value
        dp = self.get_parameter('dp').get_parameter_value().integer_value
        minDist = self.get_parameter('minDist').get_parameter_value().integer_value
        seuil_mask = self.get_parameter('seuil_mask').get_parameter_value().integer_value


        # lower boundary RED color range values; Hue (0 - 10)
        lower1 = np.array([0, sat_min, val_min])
        upper1 = np.array([hue_low_max, sat_max, val_max])

        # upper boundary RED color range values; Hue (160 - 180)
        lower2 = np.array([hue_hight_min,sat_min,val_min])
        upper2 = np.array([179,sat_max,val_max])

        img = cv2.GaussianBlur(img_origin, (5, 5), 0)
        hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)

        lower_mask = cv2.inRange(hsv, lower1, upper1)
        upper_mask = cv2.inRange(hsv, lower2, upper2)
        full_mask = lower_mask + upper_mask

        result = cv2.bitwise_and(img , img , mask=full_mask)
        img_GRAY = cv2.cvtColor(result, cv2.COLOR_BGR2GRAY)

        circles = cv2.HoughCircles(img_GRAY,cv2.HOUGH_GRADIENT,dp/10,minDist,
                        param1=60,param2=40,minRadius=min_radius,maxRadius=max_radius)
        if circles is not None  :
            nb=circles.shape[1]
            circles = np.uint16(np.around(circles))
            # Draw the circles
            for i in circles[0,:]:          
                roi = full_mask[i[1]-min_radius:i[1]+min_radius, i[0]-min_radius:i[0]+min_radius]
                average = roi.mean(axis=0).mean(axis=0)
                if average < seuil_mask :
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
        cv2.circle(img,(60,100),max_radius,(0,0,255),2)
        cv2.circle(img,(60,100),min_radius,(255,0,0),2)
        cv2.circle(img,(60,140),int((max_radius+min_radius)/2),(0,255,255),2)
        cv2.circle(img,(60+minDist,140),int((max_radius+min_radius)/2),(0,255,255),2)      

        img_msg = self.cv_bridge.cv2_to_imgmsg(img, encoding='rgb8')
        img_msg.header.frame_id = "camera_link_optical"
        self.image_publisher.publish(img_msg)
        
        img_msg = self.cv_bridge.cv2_to_imgmsg(result, encoding='rgb8')
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
