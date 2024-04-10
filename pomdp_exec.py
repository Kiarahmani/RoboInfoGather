import os

import yaml

import omnigibson as og
from omnigibson.utils.ui_utils import choose_from_options

from collections import OrderedDict
from matplotlib import pyplot as plt
import numpy as np
import cv2
from PIL import Image

from omnigibson.object_states.pose import Pose

from MCTS_Planner import Action


def planned_action_to_real_action(act):
    # Translate planned discrete action into joint vel

    # ACTIONS
    # Format: OrderedDict([('robot0', [vel , angular_vel])])
    # Example (Spin CCW) : OrderedDict([('robot0', [0 , 1])])
    # Example (Spin CW) : OrderedDict([('robot0', [0 , -1])])
    # Example (Forward) : OrderedDict([('robot0', [1 , 0])])
    # Example (Backward) : OrderedDict([('robot0', [-1 , 0])])

    # CCW Rotation
    if act == Action.R_CCW:
        return OrderedDict([('robot0', [0 , 1])])
    
    # CW Rotation
    if act == Action.R_CW:
        return OrderedDict([('robot0', [0 , -1])])

    # Move Forward
    if act == Action.M_FORWARD:
        return OrderedDict([('robot0', [1 , 0])])

    # Move Backward
    if act == Action.M_BACKWARD:
        return OrderedDict([('robot0', [-1 , 0])])

    #Observation
    if act == Action.OBS:
        return OrderedDict([('robot0', [0 , 0])])

def planner_exec(pomdp, configs):
    # Instantiate new planner
    # Can't reuse old tree since info is probably not relevant anymore????
    planner = MCTS_Planner(pomdp, configs['planner_params']['max_time'],  configs['planner_params']['max_observations'])

    best_next_node = planner.search()

    return best_next_node.inbound_act

def pomdp_exec_loop(env, pomdp, config):
    """
    Main loop for pomdp_execution
    """

    # Run until complete
    while not pomdp.enough_info():
        # Get next action
        action = planner_exec(pomdp, config)
        
        state, _, _, _ = env.step(action)

        camera_pos, camera_ori = env.robots[0]._sensors['robot0:eyes_Camera_sensor'].get_position_orientation()

        # Update POMDP
            # 1. Loop over all object types
            # 2. Get all visible voxels for that object types resolution
            # 3. Get predicted value (obs)
            # 4. Call update for that object types belief
        for obj_tp in pomdp.bel.keys():
            # Get predictions for all voxels based on observations
            vox_preds = get_vox_preds(camera_pos, camera_ori, pomdp.bel[obj_tp], obj_tp, state['robot0'], dino_model)
            pomdp.bel[obj_tp].update(vox_preds)


            # Do the same for each feature
            for feature in pomdp.bel[obj_tp].relevant_features:
                # Get predictions for all voxels based on observations
                vox_preds = get_vox_preds(camera_pos, camera_ori, pomdp.bel[obj_tp], obj_tp, state['robot0'], dino_model, feature)
                pomdp.bel[obj_tp].update(vox_preds, feature)