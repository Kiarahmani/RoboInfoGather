import numpy as np

from pomdp import *
from MCTS_planner import Loc
from observation_utils import *
from map_utils import *

def gen_prog_from_nl(nl):

def get_objects_and_features_helper(component):
    # Start with list and then unify
    if type(component) is Query:
        obj_feat = [(component.obj_tp, None)]
        obj_feat += get_objects_and_features(component.where_clause)
        return obj_feat
    elif type(component) is WhereClause:
        if component.where_tp == "feature_enum":
            return [(component.obj_tp, component.enum_feature)]

        elif component.where_tp == "feature_scalar":
            return [(component.obj_tp, component.scalar_feature)]

        elif component.where_tp == "max":
            return [(component.obj_tp, component.scalar_feature)]

        elif component.where_tp == "min":
            return [(component.obj_tp, component.scalar_feature)]"

        elif component.where_tp == "spatial_rel":
            return [(component.obj_tp2, None)]

        elif component.where_tp == "and":
            obj_feat = get_objects_and_features_helper(component.sub_where_clause[0])
            obj_feat += get_objects_and_features_helper(component.sub_where_clause[1])
            return obj_feat

        elif component.where_tp == "or":
            obj_feat = get_objects_and_features_helper(component.sub_where_clause[0])
            obj_feat += get_objects_and_features_helper(component.sub_where_clause[1])
            return obj_feat

        elif component.where_tp == "not":
            obj_feat = get_objects_and_features_helper(component.sub_where_clause[0])
            return obj_feat

        elif component.where_tp == "true":
            return []

    elif type(component) is Map or type(component) is Count:
        return get_objects_and_features_helper(component.query)

    elif type(component) is Primitives:
        if component.prim_tp == 'real':
            return []
        elif component.prim_tp == "op":
            obj_feat = get_objects_and_features_helper(component.prim_tp)
            obj_feat += get_objects_and_features_helper(component.prim_tp2)
            return obj_feat

        else:
            return get_objects_and_features_helper(component.prim_tp)

    elif type(component) is GetNth or type(component) is Aggregator:
        return get_objects_and_features_helper(component.list)

def get_objects_and_features(query):
    # Start with list and then unify
    obj_feat_list = get_objects_and_features_helper(query)

    # Unify
    obj_feat_dict = {}
    for obj, feat in obj_feat_list:
        if obj in obj_feat_dict:
            if feat not in obj_feat_dict[obj]:
                obj_feat_dict[obj].append(feat)
        else:
            obj_feat_dict[obj] = [feat]

    return obj_feat_dict

def gen_pomdp_from_query(query, pos, ori, trav_map, configs, prev_pomdp=None):
    class POMDP():
    def __init__(self, robot_init_loc, obj_tp_list, trav_map, configs, priors=None):

    theta = np.arccos(quat_to_rot(ori)[0][0])
    robot_init_loc = Loc(pos[0], pos[1], theta)

    if prev_pomdp == None:
        # Search Query and get object type list
        # (obj_tp, num, thresh, [relevant_features], priors)
        theshold = query.threshold
        num = query.limit

        obj_feat_dict = get_objects_and_features(query)

        # Make list
        obj_tp_list = []
        for obj in obj_feat_dict:
            obj_tp_list.append(obj, num, threshold, obj_feat_dict[obj], None)

        assert False # PRIORS?
        new_pomdp = POMDP(robot_init_loc, obj_tp_list, trav_map, configs)

        return new_pomdp

    else: # Build off of old pomdp
        theshold = query.threshold
        num = query.limit

        obj_feat_dict = get_objects_and_features(query)

        # Make list and unify with old
        obj_tp_list = []
        for obj in obj_feat_dict:
            if obj not in prev_pomdp.bel:
                # Get map params based off of current params and obj_tp
                map_params = get_map_params(obj, prev_pomdp.trav_map_original_size, prev_pomdp.trav_map_original_resolution)

                if priors == None:
                    prev_pomdp.bel[obj_tp] = ObjTpBel(num, threshold, map_params, configs, relevant_features)
                else:
                    prev_pomdp.bel[obj_tp] = priors

                prev_pomdp.reward_funcs[obj_tp] = RewardFunc(map_params, prev_pomdp.camaera_params, prev_pomdp.rf_params)

            else: # Check for features
                for feat in obj_feat_dict[obj]:
                    if feat not in prev_pomdp.bel[obj].feature_bels:
                        # Make a new feature belief 
                        prev_pomdp.bel[obj].feature_bels[feature] = np.copy(prev_pomdp.bel[obj].backup_p)

        return prev_pomdp


def eval_query(component, symbolic_info):
    # Set the symbolic info in the query then execute
    return component.execute(symbolic_info)