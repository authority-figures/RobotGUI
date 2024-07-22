import time
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ''))
import numpy as np
import pybullet as p
import sys
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ''))

from utils import *
from Environment.PybulletEnv import PybulletEnv
from RobotDir.Robot import Robot
from RobotDir.Machine import Machine
from RobotDir.Gripper import Gripper
import pybullet_data
from RobotDir.Camera import Camera
from Controller import Controller
import threading


class GraspSystem:
    # 创建pybullet连接
    def __init__(self,default_init=True):
        self.running = True
        self.controller = Controller()
        if default_init:
            self.default_init()

    def default_init(self):
        id_client = p.connect(p.GUI)
        p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1, shadowMapWorldSize=1, shadowMapIntensity=1, physicsClientId=id_client)
        p.setAdditionalSearchPath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '../Robot/urdf/'))
        # 设置数据搜索路径
        p.setAdditionalSearchPath(pybullet_data.getDataPath(), physicsClientId=id_client)

        planeId = p.loadURDF("plane.urdf")

        cam_urdf = "F:\\sw\\urdf_files\\camera\\urdf\\camera.urdf"
        self.camera = Robot(id_client)
        self.camera.load_urdf(fileName=cam_urdf,basePosition=(0,0,1),baseOrientation=p.getQuaternionFromEuler([1.57, 0, 0]),useFixedBase=0)


        self.env = PybulletEnv(id_client=id_client)
        robot_urdf = "F:\sw\\urdf_files\i7-nofork.SLDASM\\urdf\i7-nofork.SLDASM.urdf"

        self.env.load_robot(fileName=robot_urdf, basePosition=(-0.5, 0, 0),useFixedBase=True,
                            flags=p.URDF_USE_SELF_COLLISION, start=[0, 0, 0, 0, 0, 0])
        self.gp = Gripper(id_client)
        file_name = "F:\\sw\\urdf_files\\Gripper_1\\urdf\\Gripper_1.urdf"
        basePosition = [0, -0.3, 1]
        baseOrientation = p.getQuaternionFromEuler([0, 0, 0])
        self.gp.load_urdf(fileName=file_name, basePosition=basePosition, baseOrientation=baseOrientation, useFixedBase=0)

        self.gp.init_gripper(ifshow=True)
        constraint1 = self.bind_cam2robot(self.env.robots[0],self.camera,pos_in_robot_end_link=[0,0.0370,-0.00358], pos_in_cam_base_link=[0,0,0],ori_in_cam_base_link=[3.14,0,3.14])
        constraint2 = self.bind_gripper2robot(self.env.robots[0],self.gp)
        # constraint1 = self.bind_gripper2robot(self.env.robots[0], self.gp,[0.01,0,0],[0.01,0,0])

        self.link_c = Robot(id_client)
        self.link_c.load_urdf(fileName="F:\\sw\\urdf_files\\Link_C_18parts\\urdf\\Link_C_18parts.urdf", basePosition=[0, 0, 0.2],useFixedBase=True)

        self.T_block = Robot(id_client)
        self.T_block.load_urdf(fileName="F:\\sw\\urdf_files\\T_block\\urdf\\T_block_new.urdf",
                               basePosition=[-0.5, -0.3, 0.1], baseOrientation=[0, 0, 0, 1], useFixedBase=0)
        p.changeDynamics(
            bodyUniqueId=self.T_block.id_robot,
            linkIndex=-1,
            lateralFriction=1,
            mass=0.02
        )


        self.create_box()

        p.setGravity(0, 0, -9.8)
        self.init_robot_motor()
        self.create_virtual_cams(60)



    def reset_sys(self):
        p.resetSimulation(physicsClientId=self.env.id_client)
        p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1, shadowMapWorldSize=1, shadowMapIntensity=1,
                                   physicsClientId=self.env.id_client)
        p.setAdditionalSearchPath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '../Robot/urdf/'))
        # 设置数据搜索路径
        p.setAdditionalSearchPath(pybullet_data.getDataPath(), physicsClientId=self.env.id_client)

        planeId = p.loadURDF("plane.urdf")

        cam_urdf = "F:\\sw\\urdf_files\\camera\\urdf\\camera.urdf"
        self.camera = Robot(self.env.id_client)
        self.camera.load_urdf(fileName=cam_urdf, basePosition=(0, 0, 1),
                              baseOrientation=p.getQuaternionFromEuler([1.57, 0, 0]), useFixedBase=0)


        robot_urdf = "F:\sw\\urdf_files\i7-nofork.SLDASM\\urdf\i7-nofork.SLDASM.urdf"

        self.env.load_robot(fileName=robot_urdf, basePosition=(-0.5, 0, 0), useFixedBase=True,
                            flags=p.URDF_USE_SELF_COLLISION, start=[0, 0, 0, 0, 0, 0])
        self.gp = Gripper(self.env.id_client)
        file_name = "F:\\sw\\urdf_files\\Gripper_1\\urdf\\Gripper_1.urdf"
        basePosition = [0, -0.3, 1]
        baseOrientation = p.getQuaternionFromEuler([0, 0, 0])
        self.gp.load_urdf(fileName=file_name, basePosition=basePosition, baseOrientation=baseOrientation,
                          useFixedBase=0)

        self.gp.init_gripper(ifshow=False)
        constraint1 = self.bind_cam2robot(self.env.robots[0], self.camera, pos_in_robot_end_link=[0, 0.0370, -0.00358],
                                          pos_in_cam_base_link=[0, 0, 0], ori_in_cam_base_link=[3.14, 0, 3.14])
        constraint2 = self.bind_gripper2robot(self.env.robots[0], self.gp)
        # constraint1 = self.bind_gripper2robot(self.env.robots[0], self.gp,[0.01,0,0],[0.01,0,0])

        self.create_box()
        p.setGravity(0, 0, -9.8)
        self.init_robot_motor()
        self.create_virtual_cams(60)


    def bind_cam2robot(self,robot:Robot,camera:Robot,pos_in_robot_end_link=[0,0,0], pos_in_cam_base_link=[0,0,0],ori_in_cam_base_link=[0,0,0]):



        robot_end_mass = Robot.get_com_in_link_frame(robot.id_robot, robot.id_end_effector)
        robot_end_in_mass_sys = np.array(pos_in_robot_end_link) - np.array(robot_end_mass)

        camera_mass = Robot.get_com_in_link_frame(camera.id_robot, -1, baseFramePosition=camera.baseLinkPosition)
        camera_in_mass_sys = -np.array(camera.baseFramePosition)
        constraint_id = p.createConstraint(parentBodyUniqueId=self.env.robots[0].id_robot,
                                           parentLinkIndex=self.env.robots[0].id_end_effector,
                                           childBodyUniqueId=camera.id_robot,
                                           childLinkIndex=-1,  # 表示连接到工件的基础部分
                                           jointType=p.JOINT_FIXED,  # p.JOINT_POINT2POINT p.JOINT_REVOLUTE
                                           jointAxis=[0, 0, 0],
                                           parentFramePosition=robot_end_in_mass_sys,
                                           childFramePosition=camera_in_mass_sys,
                                           childFrameOrientation=p.getQuaternionFromEuler(ori_in_cam_base_link),
                                           )

        p.changeConstraint(constraint_id, maxForce=1e10, )
        # p.changeConstraint(constraint_id,  erp=0.2)
        for i in range(camera.id_end_effector+1):
            p.setCollisionFilterPair(camera.id_robot, robot.id_robot, i, robot.id_end_effector,
                                     0,
                                     physicsClientId=self.env.id_client)
            p.changeDynamics(camera.id_robot, i, mass=0.00001)

        p.setCollisionFilterPair(camera.id_robot, robot.id_robot, -1, robot.id_end_effector,
                                 0,
                                 physicsClientId=self.env.id_client)

        p.changeDynamics(camera.id_robot, -1, mass=0.001)

        for _ in range(10):
            p.stepSimulation()
        pass

    def create_virtual_cams(self,fps=60):
        robot_id = self.env.robots[0].id_robot
        self.left_depth_camera = Camera(robotId=robot_id,width=640,height=480,show_cv=0)
        self.left_depth_camera.focal_length_pixels = 383.3886413574219
        self.right_depth_camera = Camera(robotId=robot_id,width=640,height=480,show_cv=0)
        self.right_depth_camera.focal_length_pixels = 383.3886413574219
        self.RGB_camera = Camera(robotId=robot_id,width=640,height=480,fps=fps,show_cv=0,nearVal=0.05)
        self.right_depth_camera.focal_length_pixels = (606.8582763671875+606.9253540039062)/2
        self.RGB_camera2endEffector_pos = [-0.030383358162982828, 0.04983539700085212, -0.013351126934074722]
        pass

    def reset_cam_fps(self,fps):
        self.RGB_camera.fps = fps
        pass


    def update_cam_pos(self):
        RGB_camera_state = p.getLinkState(self.camera.id_robot,3)
        # 获取四元数
        orientation_quat = RGB_camera_state[5]
        # 将四元数转换为旋转矩阵
        rot_matrix = p.getMatrixFromQuaternion(orientation_quat)
        rot_matrix = np.array(rot_matrix).reshape(3, 3)  # 转换为3x3矩阵
        # 定义局部 z 轴向量
        local_z_axis = np.array([0, 0, 1])
        global_z_axis = rot_matrix.dot(local_z_axis)
        self.RGB_camera.updata_cam_pos_inRobotSys(RGB_camera_state[4],global_z_axis,rot_matrix)
        pass



    def bind_gripper2robot(self,robot:Robot, gripper:Gripper, pos_in_robot_end_link=[0,0,0], pos_in_gripper_base_link=[0,0,0]):
        robot_end_mass = Robot.get_com_in_link_frame(robot.id_robot, robot.id_end_effector)
        robot_end_in_mass_sys = np.array(pos_in_robot_end_link)-np.array(robot_end_mass)

        gripper_mass = Robot.get_com_in_link_frame(gripper.id_robot, -1,baseFramePosition=gripper.baseLinkPosition)
        gripper_in_mass_sys = np.array(pos_in_gripper_base_link)-np.array(gripper_mass)


        constraint_id = p.createConstraint(parentBodyUniqueId=self.env.robots[0].id_robot,
                                           parentLinkIndex=self.env.robots[0].id_end_effector,
                                           childBodyUniqueId=gripper.id_robot,
                                           childLinkIndex=-1,  # 表示连接到工件的基础部分
                                           jointType=p.JOINT_FIXED,  # p.JOINT_POINT2POINT p.JOINT_REVOLUTE
                                           jointAxis=[0, 0, 0],
                                           parentFramePosition=robot_end_in_mass_sys,
                                           childFramePosition=gripper_in_mass_sys,
                                           childFrameOrientation=p.getQuaternionFromEuler((0, 0, 0)),
                                           )


        p.changeConstraint(constraint_id,maxForce=1e8,)
        p.setCollisionFilterPair(gripper.id_robot, robot.id_robot, -1, robot.id_end_effector,
                                 0,
                                 physicsClientId=self.env.id_client)

        for _ in range(100):
            p.stepSimulation()




        return constraint_id

        pass


    def init_robot_motor(self):

        for i in range(self.env.robots[0].num_avail_joints):
            p.resetJointState(self.env.robots[0].id_robot,i,0)

        force = 10000
        mode = p.VELOCITY_CONTROL
        p.setJointMotorControl2(self.env.robots[0].id_robot, 0, mode, targetVelocity=0, force=force)
        p.setJointMotorControl2(self.env.robots[0].id_robot, 1, mode, targetVelocity=0, force=force)
        p.setJointMotorControl2(self.env.robots[0].id_robot, 2, mode, targetVelocity=0, force=force)
        p.setJointMotorControl2(self.env.robots[0].id_robot, 3, mode, targetVelocity=0, force=force)
        p.setJointMotorControl2(self.env.robots[0].id_robot, 4, mode, targetVelocity=0, force=force)
        p.setJointMotorControl2(self.env.robots[0].id_robot, 5, mode, targetVelocity=0, force=force)
        for joint in range(6):
            p.changeDynamics(self.env.robots[0].id_robot, joint, linearDamping=0.0, angularDamping=0.5)

        # for i in range(self.env.robots[0].id_end_effector):
        #     p.changeDynamics(self.env.robots[0].id_robot, i, mass=10)
        for _ in range(10):
            p.stepSimulation()
            time.sleep(1/240.)


        pass

    def print_robot_link_state(self,link_id=None):
        p.stepSimulation()
        if link_id is None:
            link_id = self.env.robots[0].id_end_effector
        state = p.getLinkState(self.env.robots[0].id_robot, link_id)
        print(f"pos:{state[4]} ori:{state[5]}")
        return state
        pass

    def print_robot_joint_values(self,):
        p.stepSimulation()
        joints = []
        for i in range(self.env.robots[0].num_all_joints):
            joint = p.getJointState(self.env.robots[0].id_robot,i)[0]
            joints.append(joint)
        print(f"joint values:{joints}")
        return joints
        pass


    def keep_joints_states(self,joint_angles,targetVelocity=0.0,positionGain=0.1,force=1000):
        for i, joint_position in enumerate(joint_angles):
            p.setJointMotorControl2(self.env.robots[0].id_robot, i, p.POSITION_CONTROL, targetPosition=joint_position,
                                    targetVelocity=targetVelocity,
                                    positionGain=positionGain,  # KP
                                    velocityGain=1,  # KD
                                    force=force,
                                    )
        pass


    def create_box(self):

        box_pos = [-0.2, -0.4, 0]
        box_ori = [0, 0, 0, 1]
        # 创建长方体的碰撞形状，设置尺寸
        collision_shape_id = p.createCollisionShape(
            shapeType=p.GEOM_BOX,
            halfExtents=[0.02, 0.02, 0.02]  # x, y, z的半长度，所以总长度分别为20mm, 20mm, 1000mm
        )

        # 创建多体，添加到仿真环境中
        self.box_id = p.createMultiBody(
            baseMass=1,  # 设置质量
            baseCollisionShapeIndex=collision_shape_id,  # 使用刚刚创建的碰撞形状
            basePosition=box_pos,  # 初始位置，基于中心点

        )

        self.gp.set_friction_coefficient(friction_coefficient=10.0)



    def grasp_T_block(self,ori_pos,tar_pos,ori_oriet=[0,0,0,1],tar_oriet=[0,0,0,1]):
        # 重力设置，使环境中的物体受重力影响
        p.setGravity(0, 0, -9.8)
        p.resetBasePositionAndOrientation(self.T_block.id_robot,ori_pos,ori_oriet)
        # p.resetBasePositionAndOrientation(self.T_block.id_robot,[-0.18,-0.18,0.165],[0, 0, -0.3824995, 0.9239557])



        self.gp.set_friction_coefficient(friction_coefficient=2)
        robot_target_pos = np.array(ori_pos) + np.array([0, 0, 0.145])
        robot_target_ori = [0, 0, 0, 1]
        target_joints = self.env.robots[0].get_state_from_ik(robot_target_pos, robot_target_ori)

        for i, joint_position in enumerate(target_joints):
            p.setJointMotorControl2(self.env.robots[0].id_robot, i, p.POSITION_CONTROL,
                                    targetPosition=joint_position,
                                    targetVelocity=0.0,
                                    positionGain=0.01,  # KP
                                    velocityGain=1,  # KD
                                    force=500,
                                    )

        state = p.getLinkState(self.env.robots[0].id_robot, self.env.robots[0].id_end_effector)[4]
        print(state)
        print(target_joints)
        self.gp.open('useVelocity', targetVelocity=10, force=10)

        # p.stepSimulation()

        self.simulate_ns(self.env.id_client, 5)
        state = self.print_robot_link_state()
        self.print_robot_joint_values()

        self.gp.close('useVelocity', targetVelocity=0.3, force=100)
        self.simulate_ns(self.env.id_client, 5)

        target_joints = self.env.robots[0].get_state_from_ik(np.array(state[0]+np.array([0,0,0.5])) ,[0,0,0,1])
        for i, joint_position in enumerate(target_joints):
            p.setJointMotorControl2(self.env.robots[0].id_robot, i, p.POSITION_CONTROL, targetPosition=joint_position,
                                    targetVelocity=0.0,
                                    positionGain=0.005,  # KP
                                    velocityGain=1,  # KD
                                    force=1000,
                                    )
        self.simulate_ns(self.env.id_client, 5)
        self.print_robot_link_state()

        robot_target_pos = np.array(tar_pos) + np.array([0, 0, 0.145])
        target_joints = self.env.robots[0].get_state_from_ik(robot_target_pos, tar_oriet)
        for i, joint_position in enumerate(target_joints):
            p.setJointMotorControl2(self.env.robots[0].id_robot, i, p.POSITION_CONTROL, targetPosition=joint_position,
                                    targetVelocity=0.0,
                                    positionGain=0.005,  # KP
                                    velocityGain=1,  # KD
                                    force=1000,
                                    )
        self.simulate_ns(self.env.id_client, 5)
        self.print_robot_link_state()

        robot_target_pos = np.array([-0.17,-0.17,0.166]) + np.array([0, 0, 0.145])
        target_joints = self.env.robots[0].get_state_from_ik(robot_target_pos, tar_oriet)
        for i, joint_position in enumerate(target_joints):
            p.setJointMotorControl2(self.env.robots[0].id_robot, i, p.POSITION_CONTROL, targetPosition=joint_position,
                                    targetVelocity=0.0,
                                    positionGain=0.005,  # KP
                                    velocityGain=1,  # KD
                                    force=1000,
                                    )

        self.simulate_ns(self.env.id_client, 5)
        # for i, joint_position in enumerate(target_joints):
        #     p.resetJointState(self.env.robots[0].id_robot, i, joint_position, )
        # self.simulate_ns(self.env.id_client, 5)
        self.print_robot_link_state()
        self.gp.open('usePosition', targetPosition=0.5, targetVelocity=0.1, force=10)
        self.simulate_ns(self.env.id_client, 1)
        robot_target_pos = np.array([-0.22, -0.22, 0.166]) + np.array([0, 0, 0.145])
        target_joints = self.env.robots[0].get_state_from_ik(robot_target_pos, tar_oriet)
        for i, joint_position in enumerate(target_joints):
            p.setJointMotorControl2(self.env.robots[0].id_robot, i, p.POSITION_CONTROL, targetPosition=joint_position,
                                    targetVelocity=0.0,
                                    positionGain=0.005,  # KP
                                    velocityGain=1,  # KD
                                    force=1000,
                                    )


        pass



    def grasp_box(self):
        # 加载地面平面
        planeId = p.loadURDF("plane.urdf")
        box_pos = [-0.2,-0.4,0]
        # 创建长方体的碰撞形状，设置尺寸
        collision_shape_id = p.createCollisionShape(
            shapeType=p.GEOM_BOX,
            halfExtents=[0.02, 0.02, 0.02]  # x, y, z的半长度，所以总长度分别为20mm, 20mm, 1000mm
        )

        # 创建多体，添加到仿真环境中
        body_id = p.createMultiBody(
            baseMass=1,  # 设置质量
            baseCollisionShapeIndex=collision_shape_id,  # 使用刚刚创建的碰撞形状
            basePosition=box_pos,  # 初始位置，基于中心点

        )

        self.gp.set_friction_coefficient(friction_coefficient=10.0)

        # self.simulate_ns(self.env.id_client,2)

        # 重力设置，使环境中的物体受重力影响
        p.setGravity(0, 0, -9.8)
        p.stepSimulation()
        state = p.getLinkState(self.env.robots[0].id_robot, self.env.robots[0].id_end_effector)
        robot_target_pos = np.array(box_pos)+np.array([0,0,0.14])
        robot_target_ori = [0,0,0,1]
        target_joints = self.env.robots[0].get_state_from_ik(robot_target_pos, robot_target_ori)
        self.gp.open('useVelocity', targetVelocity=10, force=10)

        joints = self.env.robots[0].calc_path_joints(None,None,robot_target_pos,robot_target_ori)
        inter_joints = self.env.robots[0].interpolation_path(joints,scale=10)
        self.env.robots[0].run_joints_lists(inter_joints,self.env.robots[0].id_robot,sleep_time=1/240.,KP=0.1,targetVelocity=0)

        state = p.getLinkState(self.env.robots[0].id_robot, self.env.robots[0].id_end_effector)[4]
        print(state)
        print(target_joints)

        self.print_robot_link_state()
        self.print_robot_joint_values()
        self.keep_joints_states(target_joints,positionGain=0.5)
        self.gp.close('useVelocity',targetVelocity=0.4, force=10)
        self.simulate_ns(self.env.id_client,3)

        robot_target_pos = np.array([-0.2, -0.2, 0.5])
        robot_target_ori = [0, 0, 0, 1]
        target_joints = self.env.robots[0].get_state_from_ik(robot_target_pos, robot_target_ori)
        joints = self.env.robots[0].calc_path_joints(None, None, robot_target_pos, robot_target_ori)
        inter_joints = self.env.robots[0].interpolation_path(joints, scale=20)
        self.env.robots[0].run_joints_lists(inter_joints, self.env.robots[0].id_robot, sleep_time=1 / 240., KP=0.1,
                                            targetVelocity=0)
        # for i, joint_position in enumerate(target_joints):
        #     p.setJointMotorControl2(self.env.robots[0].id_robot, i, p.POSITION_CONTROL, targetPosition=joint_position,
        #                             targetVelocity=0.0,
        #                             positionGain=0.005,  # KP
        #                             velocityGain=1,  # KD
        #                             force=1000,
        #                             )
        # self.simulate_ns(self.env.id_client,5)
        self.print_robot_link_state()
        self.keep_joints_states(target_joints, positionGain=0.5)
        # self.simulate_ns(self.env.id_client, 5)
        self.simulate_ns(self.env.id_client, 1)
        robot_target_pos = np.array([-0.2, 0.0, 0.5])
        robot_target_ori = [0, 0, 0, 1]
        target_joints = self.env.robots[0].get_state_from_ik(robot_target_pos, robot_target_ori)
        joints = self.env.robots[0].calc_path_joints(None, None, robot_target_pos, robot_target_ori)
        inter_joints = self.env.robots[0].interpolation_path(joints, scale=20)
        self.env.robots[0].run_joints_lists(inter_joints, self.env.robots[0].id_robot, sleep_time=1 / 240., KP=0.1,
                                            targetVelocity=0)
        # for i, joint_position in enumerate(target_joints):
        #     p.setJointMotorControl2(self.env.robots[0].id_robot, i, p.POSITION_CONTROL, targetPosition=joint_position,
        #                             targetVelocity=0.0,
        #                             positionGain=0.005,  # KP
        #                             velocityGain=1,  # KD
        #                             force=1000,
        #                             )

        # self.simulate_ns(self.env.id_client, 5)
        self.print_robot_link_state()
        self.keep_joints_states(target_joints, positionGain=0.5)
        self.simulate_ns(self.env.id_client, 1)

        robot_target_pos = np.array([-0.0, 0.0, 0.2])+np.array([0,0,0.14])
        robot_target_ori = [0, 0, 0, 1]
        target_joints = self.env.robots[0].get_state_from_ik(robot_target_pos, robot_target_ori)
        joints = self.env.robots[0].calc_path_joints(None, None, robot_target_pos, robot_target_ori)
        inter_joints = self.env.robots[0].interpolation_path(joints, scale=20)
        self.env.robots[0].run_joints_lists(inter_joints, self.env.robots[0].id_robot, sleep_time=1 / 240., KP=0.1,
                                            targetVelocity=0)
        # for i, joint_position in enumerate(target_joints):
        #     p.setJointMotorControl2(self.env.robots[0].id_robot, i, p.POSITION_CONTROL, targetPosition=joint_position,
        #                             targetVelocity=0.0,
        #                             positionGain=0.005,  # KP
        #                             velocityGain=1,  # KD
        #                             force=1000,
        #                             )

        # self.simulate_ns(self.env.id_client, 5)

        self.print_robot_link_state()
        self.keep_joints_states(target_joints, positionGain=0.5)
        self.simulate_ns(self.env.id_client, 2)
        self.gp.open('useVelocity', targetVelocity=1, force=10)
        self.simulate_ns(self.env.id_client, 5)


        pass

    def run(self):
        """ Main loop to run the simulation. """
        # command_thread = threading.Thread(target=self.controller.execute_commands, args=(self,))
        # command_thread.start()

        while True:
            self.controller.execute_commands(self)
            if self.controller.finished:
                break
            self.update_cam_pos()
            if self.RGB_camera.img_data:
                self.RGB_camera.display_depth_image(self.RGB_camera.img_data,0,1)
            self.sys_update()
        # command_thread.join()

    def simulate_ns(self,id_client,ntime=5):
        print(f'start to simulate for {ntime}s')
        t1 = time.time()
        while True:
            # self.update_cam_pos()
            # p.stepSimulation(physicsClientId=id_client)
            self.env.update(mode=1, robot_mode=1)
            time.sleep(1/240.)
            t2 = time.time()
            if t2-t1>ntime:
                break
        print(f'simulate for {ntime}s has been finished')
        pass



    def sys_update(self):



        self.env.update(mode=1, robot_mode=0)
        # self.camera.show_link_sys(-1, lifetime=0.1,type=1)
        # self.env.robots[0].show_link_sys(self.env.robots[0].id_end_effector,lifetime=0.1,type=1)


        time.sleep(1 / 240.)
        pass

    def env_hold_on(self):

        # for _ in range(100):
        #
        #     p.stepSimulation()
        #     time.sleep(1/240.)
        # self.grasp_box()
        # # self.grasp_T_block([-0.5,-0.5,0],tar_pos=[-0.2,-0.2,0.166],tar_oriet=[ 0, 0, 0.9236508, 0.3832351 ])
        #
        # # self.gp.reset_joint_states()
        # self.gp.open('usePosition',targetPosition=0.2, targetVelocity=0.1, force=10)


        i = 0
        while True:

            self.env.update(mode=1, robot_mode=0)
            self.camera.show_link_sys(-1, lifetime=0.1,type=1)
            self.env.robots[0].show_link_sys(self.env.robots[0].id_end_effector,lifetime=0.1,type=1)
            p.stepSimulation()
            time.sleep(1 / 240.)




if __name__ == '__main__':
    demo = GraspSystem()
    demo.env_hold_on()
    pass