import numpy as np
import pybullet as p
import time
import pybullet_data
from RobotDir.Robot import Robot
from scipy.spatial.transform import Rotation as R

def transform_to_world(link_world_position, link_world_orientation, local_position=[0.,0.,0.], local_orientation=[0.,0.,0.,1.],com_in_link=None):
    """
    将物体在连杆本体坐标系下的位姿转换为世界坐标系下的位姿。

    :param link_world_position: 连杆在世界坐标系中的位置 [x, y, z]
    :param link_world_orientation: 连杆在世界坐标系中的方向（四元数） [x, y, z, w]
    :param local_position: 物体在连杆坐标系下的局部位置 [x, y, z]
    :param local_orientation: 物体在连杆坐标系下的局部方向（四元数） [x, y, z, w]
    :return: 物体在世界坐标系下的位置和方向（四元数）
    """
    # if com_in_link is not None:
    #     local_position = np.array(local_position) - np.array(com_in_link)

    # 将连杆的四元数转换为旋转矩阵
    link_world_rot_matrix = np.array(p.getMatrixFromQuaternion(link_world_orientation)).reshape(3, 3)

    # 将局部位置转换为世界坐标系中的位置
    world_position = np.dot(link_world_rot_matrix, local_position) + link_world_position

    # 将局部方向（四元数）转换为旋转矩阵
    local_rot_matrix = np.array(p.getMatrixFromQuaternion(local_orientation)).reshape(3, 3)

    # 计算物体在世界坐标系中的旋转矩阵
    world_rot_matrix = np.dot(link_world_rot_matrix, local_rot_matrix)

    # 将世界旋转矩阵转换为四元数
    world_orientation = p.getQuaternionFromEuler(R.from_matrix(world_rot_matrix).as_euler('xyz'))

    return world_position, world_orientation


def draw(id_robot,i,life_time=0.2):
    link_state = p.getLinkState(id_robot, i, physicsClientId=0)
    joint_pos = link_state[4]
    joint_ori = link_state[5]
    rot_matrix = np.array(p.getMatrixFromQuaternion(joint_ori, physicsClientId=0)).reshape(3, 3)
    axis_length = 0.1
    x = rot_matrix @ np.array([axis_length, 0, 0])
    y = rot_matrix @ np.array([0, axis_length, 0])
    z = rot_matrix @ np.array([0, 0, axis_length])
    # X轴（红色）
    id1 = p.addUserDebugLine(joint_pos, joint_pos + x, [1, 0, 0], lineWidth=2, physicsClientId=0,lifeTime=life_time)
    # Y轴（绿色）
    id2 = p.addUserDebugLine(joint_pos, joint_pos + y, [0, 1, 0], lineWidth=2, physicsClientId=0,lifeTime=life_time)
    # Z轴（蓝色）
    id3 = p.addUserDebugLine(joint_pos, joint_pos + z, [0, 0, 1], lineWidth=2, physicsClientId=0,lifeTime=life_time)


def draw_pos(joint_pos,joint_ori,life_time=0.2):
    rot_matrix = np.array(p.getMatrixFromQuaternion(joint_ori, physicsClientId=0)).reshape(3, 3)
    axis_length = 0.1
    x = rot_matrix @ np.array([axis_length, 0, 0])
    y = rot_matrix @ np.array([0, axis_length, 0])
    z = rot_matrix @ np.array([0, 0, axis_length])
    # X轴（红色）
    id1 = p.addUserDebugLine(joint_pos, joint_pos + x, [1, 0, 0], lineWidth=2, physicsClientId=0, lifeTime=life_time)
    # Y轴（绿色）
    id2 = p.addUserDebugLine(joint_pos, joint_pos + y, [0, 1, 0], lineWidth=2, physicsClientId=0, lifeTime=life_time)
    # Z轴（蓝色）
    id3 = p.addUserDebugLine(joint_pos, joint_pos + z, [0, 0, 1], lineWidth=2, physicsClientId=0, lifeTime=life_time)
    pass




# 连接到PyBullet
p.connect(p.GUI)

# 设置搜索路径以找到URDF文件
p.setAdditionalSearchPath(pybullet_data.getDataPath())

file_name = "F:\\sw\\urdf_files\\Gripper\\urdf\\Gripper.urdf"
# 加载平面


# 加载四连杆机构的URDF文件
robot_id = p.loadURDF(fileName=file_name, basePosition=[0, 0, 0], useFixedBase=True)

# 添加闭环约束
link_l4 = 3  # link_2的索引，根据URDF加载的顺序确定
link_r4 = 9  # link_4的索引，根据URDF加载的顺序确定

# 约束链接点的世界坐标位置
pivot_in_a = np.array([-17.24, 42.65, 0])
mass_a = [-0.00792735825963712, 0.0196071278159295, 2.16840434497101E-18]
mass_b = [-0.000825228107304215, 0.00178370023886704, -0.0284876709984407]
link_state = p.getLinkState(robot_id, 3, physicsClientId=0)

pivot_in_a = transform_to_world(link_state[0],link_state[1],pivot_in_a)[0]
i_in_frame_pos_a = Robot.get_com_in_link_frame(robot_id,3)
i_in_frame_pos_a = np.array([-0.00792735825963712, 0.0196071278159295, 2.16840434497101E-18])
pivot_in_a = np.array(pivot_in_a)*0.001 - i_in_frame_pos_a
pivot_in_b = np.array([-7.5, 0., -42.55])
pivot_in_b = pivot_in_b*0.001

# 约束的旋转轴
axis_in_a = [0, 1, 0]
axis_in_b = [0, 1, 0]

# 创建点对点约束
# constraint_id = p.createConstraint(robot_id, link_l4, robot_id, -1, p.JOINT_POINT2POINT, [0, 0, 0], pivot_in_a,
#                                    pivot_in_b)
constraint_id = p.createConstraint(parentBodyUniqueId=robot_id,
                           parentLinkIndex=-1,
                           childBodyUniqueId=robot_id,
                           childLinkIndex=link_l4,  # 表示连接到工件的基础部分
                           jointType=p.JOINT_POINT2POINT,
                           jointAxis=[0, 0, 0],
                           parentFramePosition=[0, 0, 0],
                           childFramePosition=[0,0,0],
                           childFrameOrientation=p.getQuaternionFromEuler((0, 0, 0)),
                           )
# 获取机器人连杆的数量
num_joints = p.getNumJoints(robot_id)
print("Robot has {} joints/links:".format(num_joints))
for i in range(num_joints):
    joint_info = p.getJointInfo(robot_id, i)
    link_name = joint_info[12].decode("utf-8")
    joint_name = joint_info[1].decode("utf-8")
    joint_type = joint_info[2]
    parent_frame_pos = joint_info[14]
    parent_frame_orn = joint_info[15]

    link_state = p.getLinkState(robot_id, i)
    world_position = link_state[0]
    world_orientation = link_state[1]
    world_position_l = link_state[4]
    world_orientation_l = link_state[5]

    i_in_frame_pos = Robot.get_com_in_link_frame(robot_id,3)

    print("Link index:", i)
    print("  Link name:", link_name)
    print("  Joint name:", joint_name)
    print("  Joint type:", joint_type)
    print("  Parent frame position:", parent_frame_pos)
    print("  Parent frame orientation:", parent_frame_orn)
    print("  World position:", world_position)
    print("  World orientation:", world_orientation)
    print("  World position Link:", world_position_l)
    print("  World orientation Link:", world_orientation_l)
    print("  inertia in frame pos:", i_in_frame_pos)
    print()



# 开始仿真循环
while True:
    # 获取用户输入或编写自动控制逻辑
    joint_index = 1  # 控制输入关节的索引，根据URDF调整
    target_position = 0.1  # 示例值，用于输入关节的位置

    # 设置关节位置
    p.setJointMotorControl2(robot_id, joint_index, p.POSITION_CONTROL, targetPosition=target_position, force=100)

    # 步进仿真
    p.stepSimulation()
    # draw(0,3)
    link_state = p.getLinkState(robot_id, 3, physicsClientId=0)
    joint_pos = link_state[4]
    joint_ori = link_state[5]
    pos, ori = transform_to_world(joint_pos,joint_ori,[-0.01724, 0.04265, 0])
    draw_pos(pos, ori)

    draw_pos(pivot_in_b,[0,0,0,1])

    # 添加延迟以控制仿真速度
    time.sleep(1. / 240.)
