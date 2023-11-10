import numpy as np
import json
from MCTS_planner import Action

class RewardFunction_Sparse():
    def __init__(self, reward_map, x_offset, y_offset):
        self.reward_map = reward_map

        self.x_offset = x_offset
        self.y_offset = y_offset

    def eval(self, node):
        x = int(node.loc.x - self.x_offset)
        y = int(node.loc.y - self.y_offset)
        if x in range(0, self.reward_map.shape[0]) and \
          y in range(0, self.reward_map.shape[1]):
              return self.reward_map[x, y]

        return 0.0

class RewardFunction_S_Entropy():
    def __init__(self, obj_belief, obstacle_map, camera_params, rf_params):
        self.belief = obj_belief
        self.obstacle_map = obstacle_map
        self.camera_params = camera_params
        self.rf_params = rf_params

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
        map_granularity = self.rf_params['map_granularity']
        rounded_x = np.round(real_x / map_granularity) * map_granularity
        rounded_y = np.round(real_y / map_granularity) * map_granularity

        return rounded_x, rounded_y

    def get_robot_fov(self, node):
        min_angle = self.camera_params['min_angle']
        max_angle = self.camera_params['max_angle']
        min_v_dist = self.camera_params['min_visual_distance']
        max_v_dist = self.camera_params['max_visual_distance']

        angle_delta = self.rf_params['angle_delta']
        dist_delta = self.rf_params['dist_delta']

        angle = min_angle
        dist = min_v_dist

        fov = []
        while angle < max_angle:
            while dist < max_v_dist:
                # Get node that corresponds to angle and dist (relative to robot)
                x, y = self.get_new_node(node.loc, dist, angle)

                # Skip if already added
                if (x, y) in fov:
                    dist += dist_delta
                    continue

                # Check that x,y are within map bounds
                x_max = int((7-5)/0.5) #TODO: Add proper bounds based on config later
                y_max = int((73-70)/0.5)
                if int(x) not in range(0, x_max) or int(y) not in range(0, y_max):
                    break

                # Can't see through obstacles so break
                # TODO: Think about how to represent with different map
                # granularities once LIDAR setup
                if self.obstacle_map[int(x), int(y)]:
                    break

                fov.append((x,y))

                # Increment distance
                dist += dist_delta

            # Increment angle
            angle += angle_delta

        return fov

    def eval(self, node):
        # If not taking an observation action return 0
        if node.inbound_act != Action.OBS:
            return 0

        # Get grid spaces within robots FOV
        locs = self.get_robot_fov(node)

        reward = 0.0
        for (x,y) in locs:
            # TODO: Think about how to represent with different map granularities
            # once LIDAR setup is done
            p = self.belief.p[int(x), int(y)]
            reward += p * np.log(p) + (1-p)*np.log(1-p)

        reward *= -1

        return reward

def generate_reward_func(belief, ob_tp, props, func_tp='S_Entropy', exist_thresh=0.6):
    if func_tp == 'Sparse':
        ones = np.ones_like(belief.bel[ob_tp].p)
        zeros = np.zeros_like(belief.bel[ob_tp].p)
        reward_map = np.where(belief.bel[ob_tp].p > exist_thresh, ones, zeros)

        x_offset = belief.map_bounds.x_min
        y_offset = belief.map_bounds.y_min

        return RewardFunction_Sparse(reward_map, x_offset, y_offset)
    elif func_tp == 'S_Entropy':
        obj_belief = belief.bel[ob_tp]
        obstacle_map = obj_belief.p * 0 # TEMP UNTIL LIDAR OBSTACLE DETECTION

        with open('config.json', 'r') as f:
            config = json.load(f)

        camera_params = config['camera_params']
        rf_params = config['rf_params']

        return RewardFunction_S_Entropy(obj_belief, obstacle_map, camera_params, rf_params)
