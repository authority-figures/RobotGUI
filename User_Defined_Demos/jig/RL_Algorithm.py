import gym
from stable_baselines3 import DQN
from stable_baselines3.common.noise import NormalActionNoise, OrnsteinUhlenbeckActionNoise
import numpy as np
import time
from RL_env import RobotArmEnv
from stable_baselines3.common.callbacks import EvalCallback


if __name__ == '__main__':
    env = gym.make('RobotArmEnv-v0')
    # 初始化环境
    # env = RobotArmEnv()
    # env.reset()
    time.sleep(5)
    # # 设置 DQN 算法
    model = DQN("MlpPolicy", env, verbose=1,exploration_fraction=0.1, exploration_final_eps=0.02)
    eval_callback = EvalCallback(env, best_model_save_path='./logs/', log_path='./logs/', eval_freq=1000)
    # 训练模型
    model.learn(total_timesteps=50000,log_interval=10,callback=eval_callback)
    model.save("dqn_robot_arm")

    # obs = env.reset()
    # time.sleep(2)
    #
    # for i in range(1000):
    #     # action, _states = model.predict(obs, deterministic=True)
    #     action = env.action_space.sample()
    #     obs, rewards, dones, info = env.step(action)
    #     print(f'obs:{obs}, rewards:{rewards}, dones:{dones}')
    #     # env.render()
    #     if dones:
    #         obs = env.reset()