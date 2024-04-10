import roslib
import rospy
import yaml
import actionlib

import numpy as np
import pickle as pkl
import math
import json

from prolex_msgs.msg import GetCurrentLocationAction, GetCurrentLocationGoal, GetCurrentLocationFeedback, GetCurrentLocationResult, GoToAction, GoToGoal, GoToFeedback, GoToResult, IsInRoomAction, IsInRoomGoal, IsInRoomFeedback, IsInRoomResult

from prolex_msgs.msg import RetBoxAction, RetBoxGoal, RetBoxFeedback, RetBoxResult

from prolex_msgs.msg import BelPushAction, BelPushGoal, BelPushFeedback, BelPushResult
from prolex_msgs.msg import BelPullAction, BelPullGoal, BelPullFeedback, BelPullResult
from prolex_msgs.msg import GetObjsAction, GetObjsGoal, GetObjsFeedback, GetObjsResult

from MCTS_planner import Loc

from threading import Lock

class PushVoxel:
    def __init__(self, obj_tp, x, y, obs):
        self.obj_tp = obj_tp
        self.x = x
        self.y = y
        self.obs = obs

class BelUpdater:
    def __init__(self):
        #Action Clients
        self.cur_loc_client = actionlib.SimpleActionClient("/get_current_location_server",GetCurrentLocationAction)
        res = self.cur_loc_client.wait_for_server()

        print("Connected to Location Server: ", res)

        self.bel_push_client = actionlib.SimpleActionClient("/bel_push_server", BelPushAction)
        res = self.bel_push_client.wait_for_server()

        print("Connected to Belief Push Server: ", res)

       
        self.get_objs_client = actionlib.SimpleActionClient("/get_objs_server", GetObjsAction)
        res = self.get_objs_client.wait_for_server()

        print("Connected to Get Objects Server: ", res)

        # Get configs
        with open('config.json', 'r') as f:
            self.configs = json.load(f)

    def get_new_node(self, current_loc, dist, angle):
        # In robot frame: robot direction is X-axis.

        # Find the X,Y locations of the point in robot frame
        # from distance and angle
        new_x = np.cos(angle) * dist
        new_y = np.sin(angle) * dist

        # Do a rotation based on robot theta to get delta x,y in real coords
        delta_x = new_x * np.cos(current_loc.theta) - new_y * np.sin(current_loc.theta)
        delta_y = new_x * np.sin(current_loc.theta) + new_y * np.cos(current_loc.theta)

        # Get actual x and y based off of current loc
        real_x = current_loc.x + delta_x
        real_y = current_loc.y + delta_y

        # Round to map granularity and return
        map_granularity = self.configs['rf_params']['map_granularity']
        rounded_x = np.round(real_x / map_granularity) * map_granularity
        rounded_y = np.round(real_y / map_granularity) * map_granularity

        return rounded_x, rounded_y

    def get_robot_fov(self, loc):
        min_angle = self.configs['camera_params']['min_angle']
        max_angle = self.configs['camera_params']['max_angle']
        min_v_dist = self.configs['camera_params']['min_visual_distance']
        max_v_dist = self.configs['camera_params']['max_visual_distance']

        angle_delta = self.configs['rf_params']['angle_delta']
        dist_delta = self.configs['rf_params']['dist_delta']

        angle = min_angle
        dist = min_v_dist

        fov = []
        while angle < max_angle:
            while dist < max_v_dist:
                # Get node that corresponds to angle and dist (relative to robot)
                x, y = self.get_new_node(loc, dist, angle)

                # Skip if already added
                if (x, y) in fov:
                    dist += dist_delta
                    continue

                # Can't see through obstacles so break
                # TODO: Think about how to represent with different map
                # granularities once LIDAR setup
                #if self.obstacle_map[int(x), int(y)]:
                #    break

                fov.append((x,y))

                # Increment distance
                dist += dist_delta

            # Increment angle
            angle += angle_delta

        return fov

    def get_belief_vals(self):
        # Get vision Result
        get_objs_goal = GetObjsGoal(tp="new")
        self.get_objs_client.send_goal(get_objs_goal)
        self.get_objs_client.wait_for_result()

        result = self.get_objs_client.get_result()
        objs = pkl.loads(eval(result.objs))
        if len(objs.keys()) == 0: #No QR codes found
            #Get robot position
            loc_goal = GetCurrentLocationGoal()
            self.cur_loc_client.send_goal(loc_goal)
            self.cur_loc_client.wait_for_result()

            res = self.cur_loc_client.get_result()

            res = res.result.strip('()').split(' ')
            robot_loc = (float(res[0].strip(',')), float(res[1].strip(',')), float(res[2].strip(',')))

            fov = self.get_robot_fov(Loc(robot_loc[0], robot_loc[1], robot_loc[2]))


            vox_list = []
            for (x, y) in fov:
                print('Updating in FOV, (x,y) = (', x,', ', y, ')')
                vox_list.append(PushVoxel(
                    obj_tp = None,
                    x = x,
                    y = y,
                    obs = 0))

            # Return updated
            return vox_list
        else: # QR Codes found
            ret_voxes = []
            for o_id in objs:
                # Deserialize and append to list
                ret_voxes.append(PushVoxel(
                        obj_tp=objs['o_id']['obj_tp'], 
                        x=objs['o_id']['x'],
                        y=objs['o_id']['y'],
                        obs=1)
                    )

            return ret_voxes

    def run(self):
        print("Starting Belief Updater Node")
        while True:
            # Get belief vals
            obs = self.get_belief_vals()

            #Push Results
            goal = BelPushGoal()
            goal.bel_update_vals = str(pkl.dumps(obs))

            self.bel_push_client.send_goal(goal)
            self.bel_push_client.wait_for_result()

if __name__ == "__main__":
    rospy.init_node('bel_updater_client', anonymous=False)

    bel_updater_node = BelUpdater()
    bel_updater_node.run()
