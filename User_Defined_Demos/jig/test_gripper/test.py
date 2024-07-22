import pybullet as p
from pybullet_utils.bullet_client import BulletClient
import time


def get_constraint_info(self, ifshow=True):
    num_joints = p.getNumConstraints()
    all_links_info = {}
    # 遍历所有关节
    for joint_index in range(num_joints):
        # 获取关节信息
        joint_info = p.getConstraintInfo(joint_index+1)

        # 获取关联的link名称和ID
        link_name = joint_info[1]  # link名称


        link_id = joint_index  # link ID通常和关节索引相同

        # 存储信息

        all_links_info[link_id] = [link_name, joint_info[3]]

    # 打印所有link信息
    if ifshow:
        print("+" * 50)
        for link_id, link_name in all_links_info.items():
            print(f"Link ID: {link_id}, Link Name: {link_name[0]}, {link_name[1]}")
        print("-" * 50)
    pass

def get_link_info(self, ifshow=True):
    num_joints = p.getNumJoints(0)
    all_links_info = {}
    # 遍历所有关节
    for joint_index in range(num_joints):
        # 获取关节信息
        joint_info = p.getJointInfo(0, joint_index)

        # 获取关联的link名称和ID
        link_name = joint_info[12].decode('UTF-8')  # link名称


        link_id = joint_index  # link ID通常和关节索引相同

        # 存储信息

        all_links_info[link_id] = [link_name, joint_info[16]]

    # 打印所有link信息
    if ifshow:
        print("+" * 50)
        for link_id, link_name in all_links_info.items():
            print(f"Link ID: {link_id}, Link Name: {link_name[0]}, {link_name[1]}")
        print("-" * 50)
    pass


from pybullet_envs.bullet.minitaur import Minitaur

bc = BulletClient(connection_mode=p.GUI)

minitaur = Minitaur(bc, r"F:\python\RobotGUI\venv\Lib\site-packages\pybullet_data")
get_link_info([], True)
get_constraint_info([], True)
while True:
    p.setJointMotorControl2(0, 1, p.TORQUE_CONTROL, force=4, )
    # p.setJointMotorControl2(0, 1, p.VELOCITY_CONTROL, targetVelocity=0.5,force=1)
    p.stepSimulation()
    time.sleep(1/240.)