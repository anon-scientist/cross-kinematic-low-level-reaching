"""
CRoSS benchmark suite, MLF  benchmarkExample for standalone use of provided
robot manager, without RL framework
"""
from gazebo_sim.simulation.PandaRobot import PandaRobot
end=False

class RobotAction():
    def __init__(self,label,amount):
        self.label = label
        self.amount = amount

actions = []
actions.append(RobotAction("1",0.1))
actions.append(RobotAction("2",0.2))
actions.append(RobotAction("3",0.5))
actions.append(RobotAction("4",0.7))
actions.append(RobotAction("5",0.9))

robot = PandaRobot(actions)

starting_joints = robot.compute_inverse_kinematics([0.0,0.0,0.0])
if not starting_joints:
    starting_joints = robot.compute_inverse_kinematic_approx([0.0,0.0,0.0])
joint_states = list(starting_joints)

def perform_action(joint, indx):
    action = actions[indx] # select action
    joint_states[joint] = robot.get_joint_rotation_with_num(joint,action.amount)

iter = 0
while not end:
    joint = iter % 7
    if joint == 0:
        print(f"Current Pos: {robot.compute_forward_kinematic(joint_states)}. RESETTING ...")
        joint_states = list(starting_joints)
    userin = input(f"Move joint_{joint} with [12345]. Current Pos: {robot.compute_forward_kinematic(joint_states)}: ")
    if userin == "e" or userin == "exit":
        end = True
        break;
    elif int(userin) > 0 and int(userin)<=len(actions):
        perform_action(joint,int(userin)-1)
    else:
        print("Illegal Input. To exit write 'e' or 'exit'")
        continue
    iter+=1

