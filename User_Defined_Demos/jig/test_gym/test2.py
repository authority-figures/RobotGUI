from stable_baselines3 import DQN

# 创建环境
env = gym.make('TwoDSpaceEnv-v0')

# 创建DQN模型
model = DQN('MlpPolicy', env, verbose=1)

# 训练模型
model.learn(total_timesteps=10000)

# 保存模型
model.save("dqn_two_d_space")

# 加载模型
model = DQN.load("dqn_two_d_space")

# 测试模型
obs = env.reset()
for i in range(100):
    action = model.predict(obs)[0]
    obs, reward, done, info = env.step(action)
    env.render()
    if done:
        break