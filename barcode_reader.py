import roslib
import rospy
import yaml
import actionlib

import numpy as np
import pickle as pkl
import math
from sensor_msgs.msg import CompressedImage
from prolex_msgs.msg import GetCurrentLocationAction, GetCurrentLocationGoal, GetCurrentLocationFeedback, GetCurrentLocationResult, GoToAction, GoToGoal, GoToFeedback, GoToResult, IsInRoomAction, IsInRoomGoal, IsInRoomFeedback, IsInRoomResult

from prolex_msgs.msg import RetBoxAction, RetBoxGoal, RetBoxFeedback, RetBoxResult

from prolex_msgs.msg import BelPushAction, BelPushGoal, BelPushFeedback, BelPushResult
from prolex_msgs.msg import BelPullAction, BelPullGoal, BelPullFeedback, BelPullResult
from prolex_msgs.msg import GetObjsAction, GetObjsGoal, GetObjsFeedback, GetObjsResult

from threading import Lock

import cv2

class BarcodeMethod():
    def __init__(self):
        self.seen_objs = {}
        self.new_objs = {}

        # Server
        self.get_objs_server = actionlib.SimpleActionServer("/get_objs_server", GetObjsAction, self.get_objs, auto_start=False)
        self.get_objs_server.start()

        self.objs_lock = Lock()

        self.qr_reader = cv2.QRCodeDetector()

        # Barcode Subscriber
        rospy.Subscriber('/zed2i/zed_node/stereo/image_rect_color/compressed', CompressedImage, self.barcode_callback, queue_size=1)

    def barcode_callback(self, msg):
        img = msg.data
        img1 = np.frombuffer(img, np.uint8)
        img2 = cv2.imdecode(img1, cv2.IMREAD_COLOR)
        img3 = np.array(cv2.cvtColor(img2, cv2.COLOR_BGR2RGB))
        ret_qr, decoded_infos, _, _ = self.qr_reader.detectAndDecodeMulti(img3)

        if ret_qr:
            for decoded_info in decoded_infos:
                print("Raw: ", decoded_info)

                if decoded_info == '':
                    continue

                msg_obj = pkl.loads(eval(decoded_info))
                print("Decoded: ", msg_obj)
                if msg_obj['o_id'] not in self.seen_objs.keys():
                    new_obj = { 'obj_tp' : msg_obj['obj_tp'],
                            'x' : msg_obj['x'],
                            'y' : msg_obj['y']}

                    self.objs_lock.acquire()
                    self.seen_objs[msg_obj['o_id']] = new_obj
                    self.new_objs[msg_obj['o_id']] = new_obj
                    self.objs_lock.release()

    def get_objs(self, goal):
        tp = goal.tp
        r = GetObjsResult()

        if tp == "new":
            self.objs_lock.acquire()
            r.objs = str(pkl.dumps(self.new_objs))
            self.objs_lock.release()

            self.new_objs = {}
        else:
            self.objs_lock.acquire()
            r.objs = str(pkl.dumps(self.seen_objs))
            self.objs_lock.release()

        self.get_objs_server.set_succeeded(r)

if __name__ == "__main__":
    rospy.init_node('barcode_client', anonymous=False)

    barcode_node = BarcodeMethod()

    print("Barcode Reader Node Online")

    rospy.spin()
