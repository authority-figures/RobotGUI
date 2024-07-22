import time

import gym
from gym import spaces
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
class TwoDSpaceEnv(gym.Env):
    def __init__(self):
        super(TwoDSpaceEnv, self).__init__()

        # 定义动作空间：智能体可以向上、下、左、右移动
        self.action_space = spaces.Discrete(4)

        # 定义观测空间：智能体的当前位置（x, y）
        self.observation_space = spaces.Box(low=np.array([-5.0, -5.0]), high=np.array([5.0, 5.0]), dtype=np.float32)

        # 目标位置
        self.target_position = np.array([5.0, 5.0])
        self.target_radius = 0.5  # 目标点的范围半径
        # 初始化智能体位置
        self.reset()


        # 初始化绘图
        self.fig, self.ax = plt.subplots()
        self.agent_plot = None
        self.target_plot = None

    def reset(self):
        self.agent_position = np.array([np.random.uniform(0, 10), np.random.uniform(0, 10)])
        self.last_distance = None
        self.agent_state = self.target_position - self.agent_position
        # self.agent_position = np.array([9.,2.])
        return self.agent_state

    def step(self, action):
        if action == 0:  # 向上移动
            self.agent_position[1] += 0.2
        elif action == 1:  # 向下移动
            self.agent_position[1] -= 0.2
        elif action == 2:  # 向左移动
            self.agent_position[0] -= 0.2
        elif action == 3:  # 向右移动
            self.agent_position[0] += 0.2



        # 确保智能体位置在边界内
        self.agent_position = np.clip(self.agent_position, 0, 10)
        self.agent_state = self.target_position - self.agent_position

        # 计算与目标的距离作为负奖励
        distance = np.linalg.norm(self.agent_position - self.target_position)
        reward = -distance

        # 判断是否到达目标点附近
        if distance < self.target_radius:
            reward += 100  # 到达目标点附近给予高奖励
            done = True
        else:
            reward -= 10
            done = False
        reward -= 0.5

        if self.last_distance and distance<self.last_distance:
            reward += 1

        self.last_distance = distance

        return self.agent_state, reward, done, {}

    def render(self, mode='human'):
        if self.agent_plot is None:
            self.agent_plot, = self.ax.plot(self.agent_position[0], self.agent_position[1], 'bo')
            self.target_plot, = self.ax.plot(self.target_position[0], self.target_position[1], 'ro')
            self.target_circle = Circle(self.target_position, self.target_radius, color='r', fill=False)
            self.ax.add_patch(self.target_circle)
            self.ax.set_xlim(0, 10)
            self.ax.set_ylim(0, 10)
        else:
            self.agent_plot.set_data([self.agent_position[0]], [self.agent_position[1]])
        plt.pause(0.01)

    def close(self):
        plt.close()


# 注册环境
gym.envs.registration.register(
    id='TwoDSpaceEnv-v0',
    entry_point='__main__:TwoDSpaceEnv',
    max_episode_steps=100,
)

if __name__ == "__main__":
    env = gym.make('TwoDSpaceEnv-v0')
    # obs = env.reset()
    # done = False
    # total_reward = 0
    # while not done:
    #     action = env.action_space.sample()  # 随机动作
    #     obs, reward, done, info = env.step(action)
    #     total_reward += reward
    #     env.render()
    # print(f"Total reward: {total_reward}")


    from stable_baselines3 import DQN
    # # 创建DQN模型
    model = DQN('MlpPolicy', env, verbose=1)

    # 训练模型
    model.learn(total_timesteps=10000)

    # 保存模型
    model.save("dqn_two_d_space_by_vector")

    # 加载模型
    model = DQN.load("dqn_two_d_space_by_vector")

    # 测试模型

    for _ in range(10):
        obs = env.reset()
        last_obs = obs
        # obs = np.array([0,0])
        R=0
        for i in range(100):
            action = model.predict(obs)[0]
            obs, reward, done, info = env.step(action)
            R+=reward
            env.render()
            print(f'i={i} state i ={last_obs} = action i ={action} state i+1 ={obs},reward={reward},R={R}')
            time.sleep(0.1)
            last_obs = obs
            if done:
                print('finish')
                env.render()
                time.sleep(1)
                break
