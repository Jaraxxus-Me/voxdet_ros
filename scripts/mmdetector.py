#!/usr/bin/env python

# Check Pytorch installation
from logging import debug
from mmcv import image
import torch, torchvision
print(torch.__version__, torch.cuda.is_available())

# Check MMDetection installation
import mmdet
print(mmdet.__version__)

# Check mmcv installation
from mmcv.ops import get_compiling_cuda_version, get_compiler_version
print(get_compiling_cuda_version())
print(get_compiler_version())

from mmdet.apis import inference_detector, init_detector, show_result_pyplot

import os
import sys
import cv2
import numpy as np

# ROS related imports
import rospy
from std_msgs.msg import String, Header
from sensor_msgs.msg import Image
# from keyboard.msg import Key

# NOTE: 
# CvBridge meet problems since we are using python3 env
# We can do the data transformation manually
# from cv_bridge import CvBridge, CvBridgeError

from vision_msgs.msg import Detection2D, \
                            Detection2DArray, \
                            ObjectHypothesisWithPose
# from mmdetection_ros.srv import *

from mmdet.models import build_detector

import threading

# Choose to use a config and initialize the detector
CONFIG_NAME = 'VoxDet_test.py'
CONFIG_PATH = os.path.join('/ws/ROS/src/voxdet_ros/VoxDet/configs/voxdet', CONFIG_NAME)

# Setup a checkpoint file to load
MODEL_PATH = os.path.join('/ws/ROS/src/voxdet_ros/VoxDet/outputs/VoxDet_p2/iter_16876.pth')

import sys
import tty
import termios
from select import select

def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select([sys.stdin], [], [], 0.05)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

class Detector:

    def __init__(self, model):
        self.image_pub = rospy.Publisher("~debug_image", Image, queue_size=1)
        self.object_pub = rospy.Publisher("~objects", Detection2DArray, queue_size=1)
        # self.bridge = CvBridge()
        self.model = model
        self.p1_path = rospy.get_param('~p1_path')

        self._last_msg = None
        self._msg_lock = threading.Lock()

        # self._last_msg_key = 1
        # self._msg_lock_key = threading.Lock()
        self.last_key = '1'
        self.model.init(os.path.join(self.p1_path, str(self.last_key)))
        
        self._publish_rate = rospy.get_param('~publish_rate', 30)
        self._is_service = rospy.get_param('~is_service', False)
        self._visualization = rospy.get_param('~visualization', True)

    def generate_obj(self, result, id, msg):
        obj = Detection2D()
        obj.header = msg.header
        obj.source_img = msg
        result = result[0]
        obj.bbox.center.x = (result[0] + result[2]) / 2
        obj.bbox.center.y = (result[1] + result[3]) / 2
        obj.bbox.size_x = result[2] - result[0]
        obj.bbox.size_y = result[3] - result[1]

        obj_hypothesis = ObjectHypothesisWithPose()
        obj_hypothesis.id = str(id)
        obj_hypothesis.score = result[4]
        obj.results.append(obj_hypothesis)

        return obj
        

    def run(self, settings):

        if not self._is_service:
            rospy.loginfo('RUNNING MMDETECTOR AS PUBLISHER NODE')
            image_sub = rospy.Subscriber("~image", Image, self._image_callback, queue_size=1)
            rospy.loginfo('SUBSCRIBING KEYBOARD')
            # key_sub = rospy.Subscriber("/keyboard/keydown", Key, self._key_callback)
        else:
            rospy.loginfo('RUNNING MMDETECTOR AS SERVICE')
            rospy.loginfo('SETTING UP SRV')
            srv = rospy.Service('~image', mmdetSrv, self.service_handler)

        rate = rospy.Rate(self._publish_rate)
        while not rospy.is_shutdown():

            if self._msg_lock.acquire(False):
                msg = self._last_msg
                self._last_msg = None
                self._msg_lock.release()
            else:
                rate.sleep()
                continue

            key = get_key(settings)
            if key != self.last_key and key != '':
                rospy.loginfo(key)
                rospy.loginfo("Detected key, now switching to obj: {}".format(key))
                obj_id = int(key)
                self.model.init(os.path.join(self.p1_path, str(obj_id)))
                rospy.loginfo("Init done!!")
                self.last_key = key

            if msg is not None:
                objArray = Detection2DArray()
                # try:
                #     cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
                # except CvBridgeError as e:
                #     print(e)
                # NOTE: This is a way using numpy to convert manually
                im = np.frombuffer(msg.data, dtype = np.uint8).reshape(msg.height, msg.width, -1)
                # image = cv2.cvtColor(im, cv2.COLOR_BGR2RGB)
                image_np = np.asarray(im)

                # Use the detector to do inference
                # NOTE: inference_detector() is able to receive both str and ndarray
                results = inference_detector(self.model, image_np)
                # print(results)

                objArray.detections = []
                objArray.header = msg.header
                object_count = 1

                for i in range(len(results)):
                    if results[i].shape != (0, 5):
                        object_count += 1
                        objArray.detections.append(self.generate_obj(results[i], i, msg))

                if not self._is_service:
                    self.object_pub.publish(objArray)
                else:
                    rospy.loginfo('RESPONSING SERVICE')
                    return mmdetSrvResponse(objArray)

                # Visualize results
                if self._visualization:
                    # NOTE: Hack the provided visualization function by mmdetection
                    # Let's plot the result
                    # show_result_pyplot(self.model, image_np, results, score_thr=0.3)
                    # if hasattr(self.model, 'module'):
                    #     m = self.model.module
                    debug_image = self.visualize_boxes(image_np, results[0][:1])
                    # cv2.imwrite('test.jpg', debug_image)
                    # img = cv2.cvtColor(image_np, cv2.COLOR_BGR2RGB)
                    # image_out = Image()
                    # try:
                        # image_out = self.bridge.cv2_to_imgmsg(img,"bgr8")
                    # except CvBridgeError as e:
                    #     print(e)
                    # image_out.header = msg.header
                    image_out = msg
                    # NOTE: Copy other fields from msg, modify the data field manually
                    # (check the source code of cvbridge)
                    image_out.data = debug_image.tostring()

                    self.image_pub.publish(image_out)

            rate.sleep()

    def _image_callback(self, msg):
        rospy.logdebug("Get an image")
        if self._msg_lock.acquire(False):
            self._last_msg = msg
            self._msg_lock.release()

    # def _key_callback(self, msg):
    #     obj_id = self.id_switcher(msg.code)
    #     rospy.logdebug("Detected key, now switching to obj: {}".format(obj_id))
    #     if self._msg_lock_key.acquire(False):
    #         self._last_msg_key = obj_id
    #         self._msg_lock_key.release()

    def service_handler(self, request):
        return self._image_callback(request.image)

    def id_switcher(self, input):
        if input == 49:  # ASCII for "1"
            return 1

    def visualize_boxes(self, image, boxes):
        """
        Overlay bounding boxes with their scores onto the image.
        
        Parameters:
        - image: An ndarray of shape (H, W, 3)
        - boxes: An ndarray of shape (N, 5) where N is the number of boxes and each box is defined as [x1, y1, x2, y2, score]
        
        Returns:
        - An ndarray of the image with boxes overlayed.
        """
        overlayed_image = image.copy()

        for box in boxes:
            x1, y1, x2, y2, score = box
            
            # Draw the rectangle
            cv2.rectangle(overlayed_image, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)  # Green box
            
            # Put the score on the image
            label = f"{score:.2f}"  # Formatting score to 2 decimal places
            cv2.putText(overlayed_image, label, (int(x1), int(y1)-10), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0), 1)  # Red text

        return overlayed_image

def main(args):
    rospy.init_node('mmdetector')
    model = init_detector(CONFIG_PATH, MODEL_PATH, device='cuda:0')
    obj = Detector(model)
    settings = termios.tcgetattr(sys.stdin)
    # try:
    #     rospy.spin()
    # except KeyboardInterrupt:
    #     print("ShutDown")
    obj.run(settings)
    # cv2.destroyAllWindows()

if __name__=='__main__':
    main(sys.argv)