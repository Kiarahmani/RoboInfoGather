import numpy as np
import json

class ObjTpBel():
    def __init__(self, map_bounds, configs):
        self.map_bounds = map_bounds
        self.configs = configs
        
        init_alpha = self.configs['init_alpha']
        init_beta = self.configs['init_beta']
        
        x_dim = int((map_bounds.x_max - map_bounds.x_min) / self.configs['rf_params']['map_granularity'])
        y_dim = int((map_bounds.y_max - map_bounds.y_min) / self.configs['rf_params']['map_granularity'])

        print(f'Belief Created with (xdim, y_dim) = ({x_dim}, {y_dim})')

        self.alpha = np.ones((x_dim, y_dim)) * init_alpha
        self.beta = np.ones((x_dim, y_dim)) * init_alpha

        # Try sampling p rather than argmax
        self.p = np.random.beta(self.alpha, self.beta)

        print('Belief shape: ', np.shape(self.p))

    def update(self, voxel_x, voxel_y, k_inc, n_inc):
        # Update Beta Dist Params
        map_g = self.configs['rf_params']['map_granularity']
        voxel_x = int((voxel_x - self.map_bounds.x_min)/map_g)
        voxel_y = int((voxel_y - self.map_bounds.y_min)/map_g)

        # Check taht in range
        if voxel_x not in range(0, int((self.map_bounds.x_max -
            self.map_bounds.x_min)/map_g)):
            return

        if voxel_y not in range(0, int((self.map_bounds.y_max -
            self.map_bounds.y_min)/ map_g)):
            return

        print(f'Updating at Index = ({voxel_x}, {voxel_y}), with increments k = {k_inc}, n = {n_inc}')

        self.alpha[voxel_x, voxel_y] += k_inc
        self.beta[voxel_x, voxel_y] += (n_inc - k_inc)
        
        # Try sampling p rather than argmax
        alpha = self.alpha[voxel_x, voxel_y]
        beta = self.beta[voxel_x, voxel_y]
        self.p[voxel_x, voxel_y] = np.random.beta(alpha, beta)

class Belief():
    def __init__(self, robot_init_loc, obj_tp_list, map_bounds, priors=None):
        self.confidence = 0
        self.loc = robot_init_loc
        self.map_bounds = map_bounds

        with open('config.json', 'r') as f:
            self.configs = json.load(f)

        self.bel = {}
        for obj_tp in obj_tp_list:
            if priors == None:
                self.bel[obj_tp] = ObjTpBel(map_bounds, self.configs)

    def update(self, obj_tp_found, voxel_x, voxel_y, k_inc, n_inc):
        for obj_tp in self.bel.keys():
            if obj_tp == obj_tp_found or obj_tp_found == None:
                self.bel[obj_tp].update(voxel_x, voxel_y, k_inc, n_inc)
            else:
                self.bel[obj_tp].update(voxel_x, voxel_y, 1-k_inc, n_inc)

        # Update Confidence
        self.update_conf()

    def update_conf(self, tp='S_Ent'):

        # Confidence is Shannon Entropy
        if tp == 'S_Ent':
            new_conf = 0
            for obj_tp in self.bel.keys():
                p = self.bel[obj_tp].p
                print('Min p: ', np.min(p), ' Max p: ', np.max(p), ' Len: ', np.shape(p))
                assert p.all() > 0 and p.all() <= 1
                unsummed_new_conf = p * np.log(p) + (1-p)*np.log(1-p)
                assert not np.isnan(unsummed_new_conf).any()
                assert (unsummed_new_conf <= 0).all()
                unsummed_new_conf *= -1
                assert (unsummed_new_conf >= 0).all()
                new_conf += np.mean(unsummed_new_conf)

            # Normalize and invert
            new_conf = 1 - (new_conf/len(self.bel.keys()))

        else:
            # Confidence is the average variance of all voxels accross all object types
            new_conf = 0
            for obj_tp in self.bel.keys():
                alpha = self.bel[obj_tp].alpha
                beta = self.bel[obj_tp].beta
                per_vox_var = (alpha * beta)/(((alpha + beta) ** 2) * (alpha + beta + 1))

                new_conf += np.mean(per_vox_var) / len(self.bel.keys())

        self.confidence = new_conf
