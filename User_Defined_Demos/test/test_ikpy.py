import ikpy
from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink
import numpy as np
from RobotDir.Robot import Robot
from scipy.spatial.transform import Rotation as R
import time

robot_urdf = "F:\sw\\urdf_files\i7-nofork.SLDASM\\urdf\i7-nofork.SLDASM.urdf"
# 加载 URDF 文件并构建机器人运动链
robot_chain = Chain.from_urdf_file(robot_urdf)
import pybullet as p

# 设置 active_links_mask，将固定连杆设为 False
active_links_mask = [link.joint_type != 'fixed' for link in robot_chain.links]
robot_chain.active_links_mask = active_links_mask

# 打印运动链的信息
print("运动链的关节数:", len(robot_chain.links))
for i, link in enumerate(robot_chain.links):
    print(f"关节 {i} 名称: {link.name}, 类型: {link.joint_type}")

# 定义目标位置和四元数姿态
target_position = [0.25, 0.1-0.03344, 0.08]
target_orientation_quat = [ 0.0005629, -0.706825, 0.707388, 0.0005633 ]



# 将四元数转换为旋转矩阵
rotation = R.from_quat(target_orientation_quat)
rotation_matrix = rotation.as_matrix()

# 构建 4x4 变换矩阵
transform_matrix = np.eye(4)
transform_matrix[:3, :3] = rotation_matrix
transform_matrix[:3, 3] = target_position
initial_positions = np.zeros(len(robot_chain.links))
# 使用目标变换矩阵进行逆运动学计算
# joint_angles = ikpy.inverse_kinematics.inverse_kinematic_optimization(
#     chain=robot_chain,
#     target_frame=transform_matrix,
#     starting_nodes_angles=initial_positions,
#     orientation_mode='all'
# )


joint_angles = robot_chain.inverse_kinematics_frame(
    target=transform_matrix,
    initial_position=np.array([ 0.,         -0.76780261, -0.29909405, -2.44598745,  0.39766279,  2.37378243, -3.14235681]),
    orientation_mode='all',


    optimizer = 'least_squares',
)


print("逆运动学计算得到的关节角度:", joint_angles)

# 使用正运动学计算末端执行器的位置和姿态
end_effector_frame = robot_chain.forward_kinematics(joint_angles)
end_effector_position = end_effector_frame[:3, 3]
end_effector_orientation = end_effector_frame[:3, :3]
end_effector_orientation = R.from_matrix(end_effector_orientation).as_quat()
print("正运动学计算得到的末端执行器位置:", end_effector_position)
print("正运动学计算得到的末端执行器姿态:", end_effector_orientation)

id_client = p.connect(p.GUI)
robot = Robot(id_client)
robot.load_urdf(fileName=robot_urdf, basePosition=(0, 0, 0), useFixedBase=1,
                            flags=p.URDF_USE_SELF_COLLISION, )
robot.inverse_mode = 'body_sys'
'''
[ 0.,         -0.76780261, -0.29909405, -2.44598745,  0.39766279,  2.37378243, -3.14235681]
'''
# robot.set_joints_states([0.02362059, -0.33029429, -2.38706635,  0.39037985, -3.11796592,-3.17702805])
# norm_joints = joints_value = p.calculateInverseKinematics(robot.id_robot, robot.id_end_effector,
#                                                         targetPosition=target_position,
#                                                         targetOrientation=target_orientation_quat,
#                                                           )
# norm_joints = robot.get_state_from_ik(target_position,target_orientation_quat,)
show_joints = joint_angles[1:]
# show_joints = norm_joints
for i,joint in enumerate(show_joints):
    p.resetJointState(robot.id_robot, i, joint)
    pass

robot.show_link_sys(5,0,1)
print('pos:',p.getLinkState(robot.id_robot, 5)[4])
print('ori:',p.getLinkState(robot.id_robot, 5)[5])
p.stepSimulation()

time.sleep(10)

# 尝试获取多个逆解
def try_multiple_initial_positions(chain, target_position, target_orientation, num_attempts=10):
    solutions = []
    for _ in range(num_attempts):
        # 随机生成初始关节角度
        initial_positions = np.random.uniform(low=-np.pi, high=np.pi, size=len(chain.links))
        initial_positions = np.clip(initial_positions, [link.bounds[0] for link in chain.links], [link.bounds[1] for link in chain.links])
        solution = chain.inverse_kinematics(
            target_position=target_position,
            target_orientation=target_orientation,
            orientation_mode='all',
            initial_position=initial_positions
        )
        solutions.append(solution)
    return solutions

# 示例用法
solutions = try_multiple_initial_positions(robot_chain, target_position, target_orientation_vector)
print("尝试获取的多个逆解:")
for i, solution in enumerate(solutions):
    print(f"解 {i}: {solution}")