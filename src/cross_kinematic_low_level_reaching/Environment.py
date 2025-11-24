"""
$-task kinematic high-level reaching scenario
"""
import numpy as np
import argparse

from gazebo_sim.simulation.Environment import GenericEnvironment
from gazebo_sim.simulation.PandaRobot import PandaRobot as Robot
from cl_experiment.parsing import Kwarg_Parser

class Task():
    def __init__(self, name, goals):
        self.name = name
        self.goals = goals

    def get_milestone_amount(self):
        return len(self.goals)
    
class RobotAction():
    def __init__(self,label,amount):
        self.label = label
        self.amount = amount

class RobotArmEnvironment():
    def __init__(self,**kwargs) -> None:
        self.config = self.parse_args(**kwargs)

        self.training_duration = self.config.training_duration
        self.evaluation_duration = self.config.evaluation_duration
        self.max_steps_per_episode = self.config.max_steps_per_episode
        self.task_list = self.config.task_list

        # Possible Tasks
        tasks = {}
        tasks["right_down"] = Task("right_down", [[0.0,0.5,0.0]])
        tasks["right_up"] = Task("right_up", [[0.0,0.5,0.7]])
        tasks["left_down"] = Task("left_down", [[0.0,-0.5,0.3]])
        tasks["left_up"] = Task("left_up", [[0.0,-0.5,0.6]])
        tasks["front_down"] = Task("front_down", [[0.5,0.0,0.3]])
        tasks["front_up"] = Task("front_up", [[0.5,0.0,0.6]])
        tasks["back_down"] = Task("back_down", [[-0.5,0.0,0.3]])
        tasks["back_up"] = Task("back_up", [[-0.5,0.0,0.6]])
        
        ## Action space of the Robot
        actions = []
        actions.append(RobotAction("1",0.1))
        actions.append(RobotAction("2",0.2))
        actions.append(RobotAction("3",0.5))
        actions.append(RobotAction("4",0.7))
        actions.append(RobotAction("5",0.9))

        self.robot = Robot(actions)

        self.use_coords_in_obs = self.config.use_coords_in_obs == "yes"
        if self.use_coords_in_obs:
            self.observation_shape = [14]
        else:
            self.observation_shape = [11]
        self.tasks = tasks
        self.goal_discrepency_threshold = 0.2


        self.action_entries = self.robot.actions
        self.nr_actions = len(self.action_entries)

        self.step_count = 0        
        self.task_index = 0

        self.task_milestone = 0

        self.starting_joints = self.robot.compute_inverse_kinematics([0.0,0.0,0.0])
        if not self.starting_joints:
            self.starting_joints = self.robot.compute_inverse_kinematic_approx([0.0,0.0,0.0])

        self.info = {
           'input_dims': self.observation_shape,
           'number_actions': len(actions),
           'terminate_cond': 'unassigned',
        }
    
    def get_current_status(self):
        return (self.info['object'][0], self.info['terminate_cond'])
    
    def get_nr_of_tasks(self):
        return len(self.task_list)

    def get_input_dims(self):
        return self.observation_shape
    
    def get_observation(self, pos_x, pos_y, pos_z, task_id):
        if self.use_coords_in_obs:
            base_state = np.array([
                pos_x,
                pos_y,
                pos_z,
                self.tasks[task_id].goals[self.task_milestone][0],
                self.tasks[task_id].goals[self.task_milestone][1],
                self.tasks[task_id].goals[self.task_milestone][2],
                self.step_count
            ], dtype=np.float32)
        else:
            base_state = np.array([
                self.tasks[task_id].goals[self.task_milestone][0],
                self.tasks[task_id].goals[self.task_milestone][1],
                self.tasks[task_id].goals[self.task_milestone][2],
                self.step_count
            ], dtype=np.float32)

        joint_angles = np.array(self.joint_states, dtype=np.float32)

        return np.concatenate([base_state, joint_angles])
    
    def get_current_position(self):
        return self.robot.compute_forward_kinematic(self.joint_states)

    def switch(self, task_index: int) -> None:
        self.task_id = self.task_list[task_index]
        self.reset()

    def reset(self):
        self.current_name = self.task_id
        self.info['object'] = (self.current_name,)
        self.step_count = 0
        self.joint_states = list(self.starting_joints)

        current = self.get_current_position()
        state = self.get_observation(current[0],current[1],current[2],self.task_id)

        _, _, _ = self.compute_reward(state)

        return (state, self.info)

    def step(self, action_index: int):
        self.perform_action(action_index=action_index)
        self.step_count += 1

        current = self.get_current_position()
        ## Robot_X,Robot_Y,Robot_Z,Target_X,Target_Y,Target_Z,CURRENT_JOINT
        state = self.get_observation(current[0],current[1],current[2],self.task_id)

        ## compute reward
        reward, terminated, truncated = self.compute_reward(state) ;

        return state,reward,terminated,truncated, self.info ;

    def perform_action(self, action_index:int)->None:
        """ high level action execution """
        action = self.action_entries[action_index] # select action

        self.joint_states[self.step_count] = self.robot.get_joint_rotation_with_num(self.step_count,action.amount)

    def compute_reward(self, state):
        truncated = False
        terminated = False
        self.info['terminate_cond'] = "COND: Normal"

        # Euclidean Distance
        current = state[:3]
        target = state[3:6]

        magnitude = np.linalg.norm(np.array(target))
        dist = np.linalg.norm(current - target)
        normalized_distance = dist/magnitude

        reward = 1 - normalized_distance
        if self.step_count >= self.robot.joint_amount:
            terminated = True
            self.info['terminate_cond'] = "COND: GOAL NOT REACHED"
            if dist <= self.goal_discrepency_threshold:
                reward = 1
                self.info['terminate_cond'] = "COND: GOAL REACHED"
        return reward, truncated, terminated

    def parse_args(self,**kwargs):
        parser = argparse.ArgumentParser('ICRL', 'argparser of the ICRL-App.', exit_on_error=False)

        parser = Kwarg_Parser(**kwargs) ;
        # ----
        parser.add_argument("--use_coords_in_obs", type=str, default="no",required=False ) ;
        cfg,unparsed = parser.parse_known_args() ;
  
        # parse superclass params
        old_cfg = GenericEnvironment.parse_args(self, **kwargs) ;
        
        for attr in dir(old_cfg):
          # exclude magic methods and private fields
          if len(attr) > 2 and (attr[0] != "_" and attr[1] != "_"):
            setattr(cfg, attr,getattr(old_cfg, attr)) ;
        return cfg ;

    def close(self):
        pass
