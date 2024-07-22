import time
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ''))
import numpy as np
import pybullet as p
from utils import *
from Environment.PybulletEnv import PybulletEnv
from RobotDir.Robot import Robot
from RobotDir.Machine import Machine
from RobotDir.Gripper import Gripper
import pybullet_data




class RobotWithGripperDemo:
    # 创建pybullet连接
    def __init__(self):
        id_client = p.connect(p.GUI)

        p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1, shadowMapWorldSize=1, shadowMapIntensity=1, physicsClientId=id_client)
        p.setAdditionalSearchPath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '../Robot/urdf/'))
        # 设置数据搜索路径
        p.setAdditionalSearchPath(pybullet_data.getDataPath(), physicsClientId=id_client)
        self.env = PybulletEnv(id_client=id_client)

        robot_urdf = "F:\sw\\urdf_files\i7-nofork.SLDASM\\urdf\i7-nofork.SLDASM.urdf"


        self.env.load_robot(fileName=robot_urdf, basePosition=(0, 0, 0),useFixedBase=True,
                            flags=p.URDF_USE_SELF_COLLISION, start=[0, 0, 0, 0, 0, 0])

        self.gp = Gripper(id_client)
        file_name = "F:\\sw\\urdf_files\\Gripper_1\\urdf\\Gripper_1.urdf"
        basePosition = [0, -0.3, 1]
        baseOrientation = p.getQuaternionFromEuler([0, 0, 0])
        self.gp.load_urdf(fileName=file_name, basePosition=basePosition, baseOrientation=baseOrientation, useFixedBase=0)

        self.gp.init_gripper()
        constraint = self.bind_gripper2robot(self.env.robots[0],self.gp)
        # constraint1 = self.bind_gripper2robot(self.env.robots[0], self.gp,[0.01,0,0],[0.01,0,0])



        p.setCollisionFilterPair(self.gp.id_robot, self.env.robots[0].id_robot, -1, self.env.robots[0].id_end_effector, 0,
                                 physicsClientId=id_client)

        p.setGravity(0, 0, -9.8)
        self.init_robot_motor()




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

        for _ in range(10):
            p.stepSimulation()



        return constraint_id

        pass


    def init_robot_motor(self):
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


    def grasp_box(self):
        # 加载地面平面
        planeId = p.loadURDF("plane.urdf")
        box_pos = [0.2,0.2,0]
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

        robot_target_pos = np.array(box_pos)+np.array([0,0,0.14])
        robot_target_ori = [0,0,0,1]
        target_joints = self.env.robots[0].get_state_from_ik(robot_target_pos, robot_target_ori)

        for i, joint_position in enumerate(target_joints):
            p.setJointMotorControl2(self.env.robots[0].id_robot, i, p.POSITION_CONTROL, targetPosition=joint_position,
                                    targetVelocity=0.0,
                                    positionGain=0.01,  # KP
                                    velocityGain=1,  # KD
                                    force=500,
                                    )
            # p.resetJointState(self.env.robots[0].id_robot, i,joint_position)

        state = p.getLinkState(self.env.robots[0].id_robot, self.env.robots[0].id_end_effector)[4]
        print(state)
        print(target_joints)
        self.gp.open('useVelocity', targetVelocity=10, force=10)




        # p.stepSimulation()

        self.simulate_ns(self.env.id_client,5)
        self.print_robot_link_state()
        self.print_robot_joint_values()



        self.gp.close('useVelocity',targetVelocity=0.2, force=10)
        self.simulate_ns(self.env.id_client,5)
        robot_target_pos = np.array([0.2, 0.2, 0.5])
        robot_target_ori = [0, 0, 0, 1]
        target_joints = self.env.robots[0].get_state_from_ik(robot_target_pos, robot_target_ori)
        for i, joint_position in enumerate(target_joints):
            p.setJointMotorControl2(self.env.robots[0].id_robot, i, p.POSITION_CONTROL, targetPosition=joint_position,
                                    targetVelocity=0.0,
                                    positionGain=0.005,  # KP
                                    velocityGain=1,  # KD
                                    force=1000,
                                    )
        self.simulate_ns(self.env.id_client,5)
        self.print_robot_link_state()

        robot_target_pos = np.array([-0.2, 0.2, 0.5])
        robot_target_ori = [0, 0, 0, 1]
        target_joints = self.env.robots[0].get_state_from_ik(robot_target_pos, robot_target_ori)
        for i, joint_position in enumerate(target_joints):
            p.setJointMotorControl2(self.env.robots[0].id_robot, i, p.POSITION_CONTROL, targetPosition=joint_position,
                                    targetVelocity=0.0,
                                    positionGain=0.005,  # KP
                                    velocityGain=1,  # KD
                                    force=1000,
                                    )

        self.simulate_ns(self.env.id_client, 5)
        self.print_robot_link_state()

        robot_target_pos = np.array([-0.2, 0.2, 0.15])
        robot_target_ori = [0, 0, 0, 1]
        target_joints = self.env.robots[0].get_state_from_ik(robot_target_pos, robot_target_ori)
        for i, joint_position in enumerate(target_joints):
            p.setJointMotorControl2(self.env.robots[0].id_robot, i, p.POSITION_CONTROL, targetPosition=joint_position,
                                    targetVelocity=0.0,
                                    positionGain=0.005,  # KP
                                    velocityGain=1,  # KD
                                    force=1000,
                                    )
        self.simulate_ns(self.env.id_client, 5)
        self.print_robot_link_state()

        self.gp.open('useVelocity', targetVelocity=10, force=10)
        self.simulate_ns(self.env.id_client, 5)


        pass

    def simulate_ns(self,id_client,ntime=5):
        print(f'start to simulate for {ntime}s')
        t1 = time.time()
        while True:
            # p.stepSimulation(physicsClientId=id_client)
            self.env.update(mode=1, robot_mode=1)
            time.sleep(1/240.)
            t2 = time.time()
            if t2-t1>ntime:
                break
        print(f'simulate for {ntime}s has been finished')
        pass





    def env_hold_on(self):


        self.grasp_box()

        self.gp.reset_joint_states()
        p.stepSimulation()

        joint_value = self.env.robots[0].get_state_from_ik([0.2, 0.4, 0.6], [0,0,0,1])
        for i, joint_position in enumerate(joint_value):
             p.setJointMotorControl2(self.env.robots[0].id_robot, i, p.POSITION_CONTROL, targetPosition=joint_position, force=1000)
        p.stepSimulation()

        i = 0
        while True:
            # self.gp.reset_joint_states()
            # self.gp.reset_joint_states()


            if i == 0:
                self.gp.open('useVelocity',targetVelocity=10, force=10)
                print('open!')

            elif i == 500:
                self.gp.close(mode='useVelocity',targetVelocity=10, force=10)  # 'useVelocity' 'useToque' 'usePosition'
                print('close!')

            self.gp.close()
            self.env.update(mode=1, robot_mode=2)

            time.sleep(1 / 240.)
            i+=1
            if i > 1000:
                i = 0


if __name__ == '__main__':
    demo = RobotWithGripperDemo()
    demo.env_hold_on()