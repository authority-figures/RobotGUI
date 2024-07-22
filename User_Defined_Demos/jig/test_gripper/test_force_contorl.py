import pybullet as p
import pybullet_data
import time
import numpy as np

# 连接到PyBullet仿真环境
p.connect(p.GUI)

# 设置附加的搜索路径
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# 加载平面
p.loadURDF("plane.urdf")

# 加载四足机器人
quadruped = p.loadURDF("quadruped/minitaur.urdf", basePosition=[0, 0, 0.2])

# 获取所有关节的数量和ID
num_joints = p.getNumJoints(quadruped)
joint_ids = [i for i in range(num_joints)]

# 设置力矩值（假设为一个常数或随时间变化的值）
torque_value = [10.0] * num_joints

# 仿真循环
for i in range(10000):
    # 更新力矩值，例如可以根据某种控制算法或状态反馈
    torque_value = [10.0 * np.sin(i * 0.01)] * num_joints

    # 使用p.TORQUE_CONTROL控制关节
    p.setJointMotorControlArray(quadruped, joint_ids, p.TORQUE_CONTROL, forces=torque_value)

    # 步进仿真
    p.stepSimulation()
    time.sleep(1. / 240.)

# 断开连接
p.disconnect()
