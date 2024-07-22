import pybullet as p
import pybullet_data
import time

# 连接到PyBullet仿真环境
p.connect(p.GUI)

# 设置附加的搜索路径
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# 加载平面
p.loadURDF("plane.urdf")

# 创建基座立方体
base_position = [0, 0, 0.5]
base_orientation = [0, 0, 0, 1]
base_half_extents = [0.1, 0.1, 0.1]
base_visual_shape_id = p.createVisualShape(shapeType=p.GEOM_BOX, halfExtents=base_half_extents)
base_collision_shape_id = p.createCollisionShape(shapeType=p.GEOM_BOX, halfExtents=base_half_extents)
base_id = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=base_collision_shape_id,
                            baseVisualShapeIndex=base_visual_shape_id, basePosition=base_position,
                            baseOrientation=base_orientation)

# 创建动臂立方体
arm_position = [0, 0, 1.0]
arm_orientation = [0, 0, 0, 1]
arm_half_extents = [0.05, 0.05, 0.5]
arm_visual_shape_id = p.createVisualShape(shapeType=p.GEOM_BOX, halfExtents=arm_half_extents)
arm_collision_shape_id = p.createCollisionShape(shapeType=p.GEOM_BOX, halfExtents=arm_half_extents)
arm_id = p.createMultiBody(baseMass=1, baseCollisionShapeIndex=arm_collision_shape_id,
                           baseVisualShapeIndex=arm_visual_shape_id, basePosition=arm_position,
                           baseOrientation=arm_orientation)

# 定义铰链约束参数
parent_link_index = -1  # 表示连接到基座
child_link_index = -1   # 表示连接到动臂本身
joint_axis = [0, 1, 0]  # 旋转轴
parent_frame_position = [0, 0, 1]  # 父连杆坐标系中的连接点
child_frame_position = [0, 0, -0.5]  # 子连杆坐标系中的连接点

# 创建铰链约束
constraint_id = p.createConstraint(parentBodyUniqueId=base_id,
                                   parentLinkIndex=parent_link_index,
                                   childBodyUniqueId=arm_id,
                                   childLinkIndex=child_link_index,
                                   jointType=p.JOINT_REVOLUTE,
                                   jointAxis=joint_axis,
                                   parentFramePosition=parent_frame_position,
                                   childFramePosition=child_frame_position)

# 开始仿真循环
while True:
    p.stepSimulation()
    time.sleep(1./240.)

# 断开连接
p.disconnect()
