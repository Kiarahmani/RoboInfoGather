import roslib
import rospy
import yaml
import actionlib

import numpy as np
import pickle as pkl
import math

from belief import *
from RRT_motion_planner.rrt_symbolic_map import *
from bel_updater import PushVoxel

from prolex_msgs.msg import GetCurrentLocationAction, GetCurrentLocationGoal, GetCurrentLocationFeedback, GetCurrentLocationResult, GoToAction, GoToGoal, GoToFeedback, GoToResult, IsInRoomAction, IsInRoomGoal, IsInRoomFeedback, IsInRoomResult

from prolex_msgs.msg import RetBoxAction, RetBoxGoal, RetBoxFeedback, RetBoxResult
from prolex_msgs.msg import BelPushAction, BelPushGoal, BelPushFeedback, BelPushResult
from prolex_msgs.msg import BelPullAction, BelPullGoal, BelPullFeedback, BelPullResult
from threading import Lock

class BelMaintainer:
    def __init__(self, map_dims, obj_tp_list):
        #Action Servers
        self.bel_push_server = actionlib.SimpleActionServer("/bel_push_server",\
                BelPushAction, self.bel_push, auto_start=False)
        self.bel_pull_server = actionlib.SimpleActionServer("/bel_pull_server",\
                BelPullAction, self.bel_pull, auto_start=False)

        self.bel_push_server.start()
        self.bel_pull_server.start()

        # Get robot location
        self.cur_loc_client = actionlib.SimpleActionClient("/get_current_location_server",GetCurrentLocationAction)
        res = self.cur_loc_client.wait_for_server()

        #Get robot position
        loc_goal = GetCurrentLocationGoal()
        self.cur_loc_client.send_goal(loc_goal)
        self.cur_loc_client.wait_for_result()

        res = self.cur_loc_client.get_result()

        res = res.result.strip('()').split(' ')
        robot_loc = (float(res[0].strip(',')), float(res[1].strip(',')), float(res[2].strip(',')))

        # Initialize Belief
        self.belief = Belief(
                robot_loc,
                obj_tp_list,
                map_dims
                )

        self.bel_lock = Lock()

    def bel_push(self, goal):
        #Do map updates
        serialized_bel_update_vals = goal.bel_update_vals

        #Deserialize Belief Update
        bel_update_vals = pkl.loads(eval(serialized_bel_update_vals))

        #Acquire lock to make sure it's not getting corrupted by pull
        self.bel_lock.acquire()

        # Run updates
        for vox in bel_update_vals:
            self.belief.update(
                    obj_tp_found = vox.obj_tp,
                    voxel_x = vox.x,
                    voxel_y = vox.y,
                    k_inc = vox.k_inc,
                    n_inc = vox.n_inc)

        # Update Robot Loc in belief
        loc_goal = GetCurrentLocationGoal()
        self.cur_loc_client.send_goal(loc_goal)
        self.cur_loc_client.wait_for_result()

        res = self.cur_loc_client.get_result()

        res = res.result.strip('()').split(' ')
        robot_loc = (float(res[0].strip(',')), float(res[1].strip(',')), float(res[2].strip(',')))

        self.belief.robot_loc = robot_loc

        self.bel_lock.release()
        self.bel_push_server.set_succeeded(True)

        print(f'Robot Location: {self.belief.robot_loc}')
        print(f'Confidence: {self.belief.confidence}')

    def bel_pull(self, goal):
        print("Pulling Belief: ", self.belief)
        #Acquire lock to make sure it's not getting corrupted by pull
        self.bel_lock.acquire()

        #Compose Result Message
        serialized_bel = str(pkl.dumps(self.belief))

        self.bel_lock.release()

        r = BelPullResult()

        r.bel = serialized_bel

        self.bel_pull_server.set_succeeded(r)

if __name__ == "__main__":
    rospy.init_node('bel_maintainer')

    test_map_dims = map_dims(x_min=5, x_max=7, y_min=70, y_max=73)

    test_obj_tp_list = ['Chair']

    bel_maintainer = BelMaintainer(test_map_dims, test_obj_tp_list)

    print("Belief Maintainer Online")

    rospy.spin()
