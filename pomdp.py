from object_belief import *
from reward_func import *
from map_utils import *

class POMDP():
    def __init__(self, robot_init_loc, obj_tp_list, trav_map, configs):
        self.prior_beliefs = []
        assert False # Above ??
        self.loc = robot_init_loc
        self.trav_map = trav_map
        self.configs = configs

        # Calculate originial map params so that we can pass to reward and belief classes
        self.trav_map_original_size = trav_map.trav_map_original_size
        self.trav_map_original_resolution = trav_map.map_default_resolution

        self.bel = {}
        self.reward_funcs = {}
        self.camaera_params = configs["camera_params"]
        self.rf_params = configs["rf_params"]
        for obj_tp, num, thresh, relevant_features, priors in obj_tp_list:
            # Get map params based off of current params and obj_tp
            self.map_params = get_map_params(obj_tp, self.trav_map_original_size, self.trav_map_original_resolution)

            if priors == None:
                self.bel[obj_tp] = ObjTpBel(num, threshold, self.map_params, self.configs, relevant_features)
            else:
                self.bel[obj_tp] = priors

            self.reward_funcs[obj_tp] = RewardFunc(map_params, self.camaera_params, self.rf_params)

    def eval_reward(self, obstacle_map, root, node):
        reward = 0
        for obj_tp in self.reward_funcs.keys():
            reward += self.reward_funcs[obj_tp].eval(self.bel[obj_tp].p, obstacle_map, root, node)

    def enough_info(self):
        self.prior_beliefs.append(self.bel)

        # Pop if over length
        while len(self.prior_beliefs) > self.configs['bel_params']['akld_hist_len']:
            self.prior_beliefs.pop(0)

        # Compute AKLD
        k = len(self.prior_beliefs)
        akld = 0
        found_all_obj = True
        for obj_tp in self.bel.keys():
            for i in range(k):
                for j in range(k):
                    akld += kl(self.prior_beliefs[i][obj_tp].p, self.prior_beliefs[j][obj_tp].p)

            # Check if enough instances of all object types
            num_found = np.sum(np.where(self.bel[obj_tp].p >= self.bel[obj_tp].threshold, 1, 0))
            if num_found < self.bel[obj_tp].num:
                found_all_obj = False


        akld = akld / (k * (k-1))

        # Check akld and num found
        if akld <= self.configs['bel_params']['akld_stop'] or found_all_obj:
            return True

        return False

    def kl(self, p, q):
        return np.sum(np.where(p != 0, p*np.log(p/q), 0))