import time

import gym
from gym import spaces
import pybullet as p
import pybullet_data
import numpy as np
import cv2
import os
from combined_sys import GraspSystem
import threading
from Controller import Controller,Command
import multiprocessing
from multiprocessing import Process,Queue


def run_controller(grasp_system):
    grasp_system.run()
    # p.disconnect(grasp_system.connection_id)

def run_simulation(command_queue,data_queue):
    grasp_system = GraspSystem()
    grasp_system.reset_sys()
    grasp_system.reset_cam_fps(30)
    grasp_system.data_queue = data_queue
    # 开始仿真在一个新线程
    thread = threading.Thread(target=run_controller, args=(grasp_system,))
    thread.start()
    while True:
        if not command_queue.empty():
            command = command_queue.get()

            grasp_system.controller.add_command(command)
            time.sleep(0.1)
            if command.type == "terminate":
                break
                pass

    thread.join()

class RobotArmEnv(gym.Env):
    metadata = {'render.modes': ['human']}

    def __init__(self):
        super(RobotArmEnv, self).__init__()
        self.command_queue = Queue()
        self.data_queue = Queue()
        self.action_space = spaces.Discrete(6)
        # Example for using image as input (observation space):
        self.observation_space = spaces.Box(low=np.array([-0.21,-0.21,-0.11]), high=np.array([0.21,0.21,0.11]), dtype=np.float32)

        self.base_pos = np.array([-0.5,-0.3,0.2])
        self.state = np.array([0.0, 0.0, 0.0])
        self.target_position = np.array([-0.5, -0.32, 0.2])
        self.stepSize = 0.005
        self.process = Process(target=run_simulation, args=(self.command_queue,self.data_queue))
        self.process.start()
        # 初始化 PyBullet

    def run_simulation(self, command_queue,data_queue):
        grasp_system = GraspSystem()
        grasp_system.reset_cam_fps(30)
        grasp_system.data_queue = data_queue

        # 开始仿真在一个新线程
        thread = threading.Thread(target=run_controller, args=(grasp_system,))
        thread.start()
        while True:
            if not command_queue.empty():
                command = command_queue.get()

                grasp_system.controller.add_command(command)
                time.sleep(0.1)
                if command.type == "terminate":
                    break
                    pass

        thread.join()

    def reset(self):
        random_pos = np.random.uniform(-0.1, 0.1, size=(3,))
        self.last_action = None
        self.last_error = None
        self.last_state = None
        self.R=0
        self.current_step = 0

        new_pos = random_pos + self.base_pos
        print(f'before reset new pos is{new_pos}')
        reset_command = Command('reset',origin_pos=new_pos,box_pos=self.target_position, parallel=True, parallel_id=-1)
        self.command_queue.put(reset_command)
        # time.sleep(1) # 等待清理队列后重新添加
        state = self.get_state(reset_command.id)
        print(f'reset with pos:{self.target_position-state}')
        return state
        # p.resetSimulation()
        # p.setGravity(0, 0, -10)
        # self.robot_id = p.loadURDF("kuka_iiwa/model.urdf", useFixedBase=True)
        # return self.get_observation()

    def step(self, action):
        self.current_step += 1
        if action == 0:  # 前进
            command = Command('move_by_delta', delta_pos=[self.stepSize,0,0], scale=10)
            self.command_queue.put(command)
            self.command_queue.put(Command('get_end_effector_data',parallel=False,parallel_id=-1))
            data = self.data_queue.get()
            pos = np.array(data[0])
            self.state = self.target_position-pos

        elif action == 1:  # 后退
            self.command_queue.put(Command('move_by_delta', delta_pos=[-self.stepSize,0,0], scale=10))
            self.command_queue.put(Command('get_end_effector_data', parallel=False, parallel_id=-1))
            data = self.data_queue.get()
            pos = np.array(data[0])
            self.state = self.target_position-pos
        elif action == 2:  # 左
            self.command_queue.put(Command('move_by_delta', delta_pos=[0.,self.stepSize,0], scale=10))
            self.command_queue.put(Command('get_end_effector_data', parallel=False, parallel_id=-1))
            data = self.data_queue.get()
            pos = np.array(data[0])
            self.state = self.target_position-pos
        elif action == 3:  # 右
            self.command_queue.put(Command('move_by_delta', delta_pos=[0.,-self.stepSize,0], scale=10))
            self.command_queue.put(Command('get_end_effector_data', parallel=False, parallel_id=-1))
            data = self.data_queue.get()
            pos = np.array(data[0])
            self.state = self.target_position-pos
        elif action == 4:  # 上
            self.command_queue.put(Command('move_by_delta', delta_pos=[0.,0,self.stepSize], scale=10))
            self.command_queue.put(Command('get_end_effector_data', parallel=False, parallel_id=-1))
            data = self.data_queue.get()
            pos = np.array(data[0])
            self.state = self.target_position-pos
        elif action == 5:  # 下
            self.command_queue.put(Command('move_by_delta', delta_pos=[0.,0,-self.stepSize], scale=10))
            self.command_queue.put(Command('get_end_effector_data', parallel=False, parallel_id=-1))
            data = self.data_queue.get()
            pos = np.array(data[0])
            self.state = self.target_position-pos

        self.this_action = action

        # 计算奖励
        reward = self.calculate_reward(self.state)

        done,result = self.judge_done(self.state)
        if done:
            if result:
                reward += 10000
            else:
                reward -= -10000

        self.last_action= action

        self.R+=reward
        print(f'i={self.current_step} state i ={self.last_state} = action i ={action} state i+1 ={self.state},reward={reward},R={self.R}')
        self.last_state = self.state
        return self.state, reward, done, {}

    def judge_done(self,state):
        error = state
        xy_error = np.linalg.norm(error[:2])
        z_error = np.linalg.norm(error[2])
        if xy_error<=0.05 and z_error<=0.05:
            return True,True
        elif xy_error>0.2 or z_error>0.1:
            return True,False
        else:
            return False,False
        pass



    def render(self, mode='human'):
        pass

    def close(self):
        p.disconnect()

    def calculate_reward(self,state):

        error = state
        xy_error = np.linalg.norm(error[:2])
        z_error = np.linalg.norm(error[2])
        distance = np.linalg.norm(error)

        reward1 = 0
        if self.last_error:
            if self.last_error>distance:
                reward1 = 2


        reward2 = -distance**2*100

        reward = reward2 + reward1 -1  # 奖励是负的距离
        # reward = -distance**3*1000

        self.last_error = distance
        return reward

    def check_if_done(self):
        # 检查任务是否完成
        return False

    def get_state(self,parallel_id=-1):
        self.command_queue.put(Command('get_end_effector_data', parallel=False, parallel_id=parallel_id))
        data = self.data_queue.get()
        pos = np.array(data[0])
        self.state = self.target_position - pos
        return self.state
        pass



def main():
    env = RobotArmEnv()
    # observation = env.reset()
    # for _ in range(100):
    #     action = env.action_space.sample()  # 随机选择一个动作
    #     observation, reward, done, info = env.step(action)
    #     if done:
    #         observation = env.reset()
    # env.close()

    pass


# 注册环境
gym.envs.registration.register(
    id='RobotArmEnv-v0',
    entry_point='__main__:RobotArmEnv',
    max_episode_steps=100,
)


if __name__ == '__main__':
    env = RobotArmEnv()
    env.reset_env()
    pass

