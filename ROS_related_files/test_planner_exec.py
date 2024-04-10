import rospy
import actionlib
from MCTS_planner import *
from reward_func_gen import *

from prolex_msgs.msg import BelPullAction, BelPullFeedback, BelPullGoal, BelPullResult
from prolex_msgs.msg import GetObjsAction, GetObjsGoal, GetObjsResult

import pickle as pkl

def found(obj_tp, obj_pull_client):
    get_objs_goal = GetObjsGoal()
    get_objs_goal.tp = 'new'

    obj_pull_client.send_goal(get_objs_goal)
    obj_pull_client.wait_for_result()

    new_objs = pkl.loads(eval(obj_pull_client.get_result().objs))

    if not new_objs:
        return False, None

    for obj in new_objs:
        if obj_tp == new_objs[obj]['obj_tp']:
            return True, {obj : new_objs[obj]}


rospy.init_node('test')

# Connect to Belief Pulling server
bel_pull_client = actionlib.SimpleActionClient('/bel_pull_server', BelPullAction)
res = bel_pull_client.wait_for_server()
print("Connected to Belief Pulling Server: ", res)

# Connect to object pulling server
obj_pull_client = actionlib.SimpleActionClient('/get_objs_server', GetObjsAction)
res = obj_pull_client.wait_for_server()
print("Connected to Object Pulling Server: ", res)

success = False
succ_find = False

# Replan until found
while not success or not succ_find:
    # Repull Belief each iteration to make sure it's up to date
    bel_pull_goal = BelPullGoal()
    bel_pull_client.send_goal(bel_pull_goal)
    bel_pull_client.wait_for_result()
    res = bel_pull_client.get_result()
    belief = pkl.loads(eval(bel_pull_client.get_result().bel))

    # Regenerate Reward each iteration to make sure it matches new belief
    reward_func = generate_reward_func(belief, 'Chair', props=None, func_tp='S_Entropy', exist_thresh=0.6)

    # Call planner exec
    success = planner_exec(belief, reward_func)

    if success == 'Replan':
        success = False

    elif success == 'Not Found':
        print('Not found')
        break

    print("Done Motion Plan? ", success)

    # Check if Found
    #succ_find, new_obj = found('Chair', obj_pull_client)
    #print(new_obj)
