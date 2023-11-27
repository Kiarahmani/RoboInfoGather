import math
import json
import time
import random

import actionlib

from enum import Enum

from prolex_msgs.msg import GoToAction, GoToGoal, GoToFeedback, GoToResult

class Loc():
    def __init__(self, x, y, theta=None):
        self.x = x
        self.y = y
        self.theta = theta

class Action(Enum):
    M_LEFT = 1
    M_RIGHT = 2
    M_DOWN = 3
    M_UP = 4
    M_LEFTDOWN = 5
    M_LEFTUP = 6
    M_RIGHTDOWN = 7
    M_RIGHTUP = 8
    L_N = 9
    L_NE = 10
    L_E = 11
    L_SE = 12
    L_S = 13
    L_SW = 14
    L_W = 15
    L_NW = 16
    OBS = 17

class MCTS_Tree_Node():
    def __init__(self, loc, map_bounds, obstacle_map, num_prev_obs, max_obs,
            map_g, parent=None, children=[], inbound_act=None, terminal=False):
        self.loc = loc
        self.map_bounds = map_bounds

        self.obstacle_map = obstacle_map

        self.num_prev_obs = num_prev_obs
        self.max_obs = max_obs

        # Map Granularity
        self.map_g = map_g

        self.children = children
        self.parent = parent
        self.inbound_act = inbound_act
        self.terminal = terminal

        self.total_rewards = 0
        self.visits = 1

        self.max_children = len(Action)

        # Find number of illegal actions and subtract from max children
        # Robot Must Start on Map Legally****
        for action in Action:
            if not self.legal(action):
                self.max_children -= 1

    def unvisited_child(self):
        avail_act_list = []
        for act in Action:
            found = False
            for child in self.children:
                if child.inbound_act == act:
                    found = True
                    break

            if not found and self.legal(act):
                avail_act_list.append(act)

        # Randomly select from available actions
        action = random.choice(avail_act_list)

        # Make new node
        new_loc = self.get_loc(act)
        
        # Increment number of previous observations if current node is an
        # Observe action
        new_num_prev_obs = self.num_prev_obs 
        if self.inbound_act == Action.OBS:
            new_num_prev_obs += 1

        child = MCTS_Tree_Node(
                loc = new_loc,
                map_bounds = self.map_bounds,
                obstacle_map = self.obstacle_map,
                num_prev_obs = new_num_prev_obs,
                max_obs = self.max_obs,
                map_g = self.map_g,
                parent = self,
                children = [],
                inbound_act = act,
                terminal = False)

        child.terminal = child.eval_terminal()

        # Add child to children and return
        self.children.append(child)
        return child

    def eval_terminal(self):
        # TODO THINK ABOUT THIS MORE
        return (self.num_prev_obs == (self.max_obs - 1)) and self.inbound_act == Action.OBS

    def legal(self, act):
        new_loc = self.get_loc(act)

        nl_x = int((new_loc.x - self.map_bounds.x_min) / self.map_g)
        nl_y = int((new_loc.y - self.map_bounds.y_min) / self.map_g)

        x_max = int((self.map_bounds.x_max - self.map_bounds.x_min) / self.map_g)
        y_max = int((self.map_bounds.y_max - self.map_bounds.y_min) / self.map_g)

        # Check that new location is within the map bounds
        if nl_x not in range(0, x_max) or nl_y not in range(0, y_max):
            return False
        
        # Check if new location would cause a collision
        if self.obstacle_map[nl_x, nl_y]:
            return False

        return True

    def get_loc(self, act):
        if act == Action.M_LEFT:
            return Loc(self.loc.x - 1, self.loc.y, self.loc.theta)
        elif act == Action.M_RIGHT:
            return Loc(self.loc.x + 1, self.loc.y, self.loc.theta)
        elif act == Action.M_DOWN:
            return Loc(self.loc.x, self.loc.y - 1, self.loc.theta)
        elif act == Action.M_UP:
            return Loc(self.loc.x, self.loc.y + 1, self.loc.theta)
        elif act == Action.M_LEFTDOWN:
            return Loc(self.loc.x - 1, self.loc.y - 1, self.loc.theta)
        elif act == Action.M_LEFTUP:
            return Loc(self.loc.x - 1, self.loc.y + 1, self.loc.theta)
        elif act == Action.M_RIGHTDOWN:
            return Loc(self.loc.x + 1, self.loc.y - 1, self.loc.theta)
        elif act == Action.M_RIGHTUP:
            return Loc(self.loc.x + 1, self.loc.y + 1, self.loc.theta)
        elif act == Action.L_N:
            return Loc(self.loc.x, self.loc.y, math.radians(0))
        elif act == Action.L_NE:
            return Loc(self.loc.x, self.loc.y, math.radians(45))
        elif act == Action.L_E:
            return Loc(self.loc.x, self.loc.y, math.radians(90))
        elif act == Action.L_SE:
            return Loc(self.loc.x, self.loc.y, math.radians(135))
        elif act == Action.L_S:
            return Loc(self.loc.x, self.loc.y, math.radians(180))
        elif act == Action.L_SW:
            return Loc(self.loc.x, self.loc.y, math.radians(225))
        elif act == Action.L_W:
            return Loc(self.loc.x, self.loc.y, math.radians(270))
        elif act == Action.L_NW:
            return Loc(self.loc.x, self.loc.y, math.radians(315))
        elif act == Action.OBS:
            return Loc(self.loc.x, self.loc.y, self.loc.theta)


    def update(self, reward):
        self.total_rewards += reward
        self.visits += 1

class MCTS_Planner():
    def __init__(self, 
            belief,
            reward_func,
            max_time,
            max_obs,
            epsilon=1e-2,
            rollout_policy="Random",
            max_rollout_depth=300):

        root_loc = Loc(int(belief.loc[0]), int(belief.loc[1]), belief.loc[2])

        with open('config.json', 'r') as f:
            self.config = json.load(f)

        # TODO Add Belief to Obstacle map once done with LIDAR implementation
        obstacle_map = belief.bel[list(belief.bel.keys())[0]].p * 0
        self.root = MCTS_Tree_Node(root_loc, belief.map_bounds, obstacle_map, 0,
                max_obs, map_g=self.config['rf_params']['map_granularity'])

        self.belief = belief
        self.reward_func = reward_func

        self.max_time = max_time
        self.epsilon = epsilon

        self.rollout_policy_tp = rollout_policy
        self.max_rollout_depth = max_rollout_depth

    def search(self):
        start_time = time.time()

        while (time.time() - start_time) < self.max_time:
            leaf = self.traverse(self.root)
            sim_reward = self.rollout(leaf)
            self.backpropogate(leaf, sim_reward)

        return self.best_child(self.root)

    def traverse(self, node):
        print(len(node.children), node.max_children)
        while len(node.children) == node.max_children:
            node = self.best_ucb(node)

        if node.terminal:
            return node
        else:
            return node.unvisited_child()

    def rollout(self, node):
        terminal = node.terminal
        depth = 0
        while not terminal and depth < self.max_rollout_depth:
            node = self.rollout_policy(node)
            terminal = node.terminal
            
            depth += 1

        return self.reward_func.eval(self.root, node)

    def rollout_policy(self, node):
        if self.rollout_policy_tp == "Random":
            # Select random action until legal discovered
            legal = False
            while not legal:
                act = random.choice(Action)
                legal = node.legal(act)

            # Make new child
            new_loc = node.get_loc(act)

            # Increment number of previous observations if current node is an
            # Observe action
            new_num_prev_obs = node.num_prev_obs 
            if node.inbound_act == Action.OBS:
                new_num_prev_obs += 1

            child = MCTS_Tree_Node(
                    loc = new_loc,
                    map_bounds = node.map_bounds,
                    obstacle_map = node.obstacle_map,
                    num_prev_obs = new_num_prev_obs,
                    max_obs = node.max_obs,
                    map_g = node.map_g,
                    parent = node,
                    children = [],
                    inbound_act = act,
                    terminal = False)

            child.terminal = child.eval_terminal()

            # Append child to current node and return
            node.children.append(child)
            return child
        else:
            assert False # No others implemented
            return None

    def backpropogate(self, node, reward):
        # Return at root
        if node.parent == None:
            return
        
        node.update(reward)

        self.backpropogate(node.parent, reward)

    def best_ucb(self, node):
        best_ucb_val = 0
        best_child = node.children[0]
        for child in node.children:
            ucb_val = self.calculate_ucb(child)
            if ucb_val > best_ucb_val:
                best_ucb_val = ucb_val
                best_child = child

        return best_child

    def calculate_ucb(self, node):
        mean = float(node.total_rewards)/node.visits
        explore_bonus = self.epsilon * math.sqrt(math.log(node.parent.visits)/node.visits)

        return mean + explore_bonus

    def best_child(self, node):
        most_visits = 0
        best_child = node.children[0]
        for child in node.children:
            if child.visits > most_visits:
                most_visits = child.visits
                best_child = child

        return best_child

def planner_exec(belief, reward_func, goto_client=None):
    # Need global to check terminal
    global global_reward_func
    global_reward_func = reward_func
    
    # Instantiate new planner
    # Can't reuse old tree since info is probably not relevant anymore????
    planner = MCTS_Planner(belief, reward_func, max_time=5.0, max_obs=1)

    best_next_node = planner.search()

    # Get terminal view node
    path = []
    path.append(best_next_node)
    while len(best_next_node.children) > 0:
        best_next_node = planner.best_child(best_next_node)
        path.append(best_next_node)

    # Find particular node in path that we want to use
    # TODO: Update this to search all observations if max_obs is greater than 1
    view_node = path[-1]

    # Execute path to get to next node if low overall map confidence
    # Execute path to get to some node under best_next_node (longer path) if high overall confidence
        # With this method we can serach longer horizons with MCTS
        # However when to replan? Say we are approaching location and we can already tell that the object isn't there?
            # Perhaps while updating the belief we keep evaluating that location on the reward function? If it falls below 
            # some threshold replan?

    # If we modify such that no nodes are terminal, MCTS is more likely to search in direciton where more objects
    # are likely to be found. I.e. only consider rollout depth, not termination of nodes. Collect rewards along the way?
        # Is this behaviour automatic in UCB selection policy?

    print("Plan Found, View Node: ", view_node.loc.x, view_node.loc.y,
            math.degrees(view_node.loc.theta), ' Locs Considered: ', len(path))

    print("Inbound Action: ", view_node.inbound_act)
    print("Parent Children Num: ", len(view_node.parent.children))

    # Set up Goto Client if None
    goto_client = actionlib.SimpleActionClient("/go_to_server", GoToAction)
    goto_client.wait_for_server()

    # Exec goto
    next_best_view = f'{view_node.loc.x}, {view_node.loc.y},{view_node.loc.theta}'
    res = goto_client.wait_for_server()
    goto_goal = GoToGoal(location=next_best_view)
    goto_client.send_goal(goto_goal)
    goto_client.wait_for_result()

    # Replan loop (recursive)
    # TODO: Figure out success conditions
    success = False
    if not success and belief.confidence < 0.95: # TODO: Get actual conf thresh
        return 'Replan'

    return 'Not Found'
