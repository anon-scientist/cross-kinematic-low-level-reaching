"""
CRoSS benchmark suite, HLR  benchmark Example for standalone use of provided
robot manager, without RL framework
"""

from src.cross_kinematic_low_level_reaching.Environment import RobotArmEnvironment

end=False

args_dict = {"task_list":["right_up","right_down"],
             "use_coords_in_obs":"yes"}

env = RobotArmEnvironment(**args_dict)

print("Switching to Task 0: ", env.task_list[0])
env.switch(0) ;

iter = 0
while not end:
    joint=iter%7
    if joint == 0: env.reset()
    action_index = 2 # For the possible actions look in Environment.py
    obs, reward, terminated, truncated, info = env.step(action_index)
    print(f"JOINT_{joint} with action {action_index} -> State: {obs}")

    if joint == 6:
        print("Reward:", reward)

    if iter >= 13:
        end=True
        break
    iter+=1