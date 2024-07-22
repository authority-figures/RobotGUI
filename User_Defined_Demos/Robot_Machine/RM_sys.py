import time
import pybullet as p
from utils import *
from Environment.PybulletEnv import PybulletEnv
from RobotDir.Robot import Robot
from RobotDir.Machine import Machine
from RobotDir.Gripper import Gripper
from scipy.spatial.transform import Rotation as R
from RM_Controller import RM_Controller as Controller
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ''))
import re
import copy
from tqdm import tqdm
from pyquaternion import Quaternion
from RobotDir.utils.my_utils import invert_quaternion

class RM_sys:
    # 创建pybullet连接
    def __init__(self,default_init=True):

        self.running = True
        self.controller = Controller()
        self.step_time = 1/ 240.
        if default_init:
            self.default_init()

    def default_init(self):
        self.id_client = p.connect(p.GUI)
        p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1, shadowMapWorldSize=1, shadowMapIntensity=1,
                                   physicsClientId=self.id_client)
        p.setAdditionalSearchPath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '../RobotDir/urdf/'))
        self.env = PybulletEnv(id_client=self.id_client)
        workpiece_urdf = r"F:\sw\urdf_files\6061_simple\urdf\6061_simple.urdf"
        machine_file_name = "F:\sw\\urdf_files\c501-simple.SLDASM\\urdf\c501-simple.SLDASM.urdf"
        # robot_urdf = "F:\sw\\urdf_files\i7-nofork.SLDASM\\urdf\i7-nofork.SLDASM.urdf"
        robot_urdf = "F:\sw\\urdf_files\Aubo_i3\\urdf\Aubo_i3.urdf"
        self.machine = Machine(self.id_client)
        self.machine.load_urdf(fileName=machine_file_name, basePosition=(0, 0, 0), useFixedBase=1, flags=0, )
        self.init_machine(type='p')
        self.workpiece_pose = [0.0, 0.2, 0.02]
        orientation = [0.0, 0.0, 0]
        quaternion = p.getQuaternionFromEuler(orientation)
        self.workpiece_orientation = quaternion
        self.workpiece_id = self.machine.add_workpiece_to_machine(workpiece_urdf, position=self.workpiece_pose,
                                                                  orientation=orientation)

        self.env.load_robot(fileName=robot_urdf, basePosition=(-0.15, -0.15, 0.7), useFixedBase=0,
                            # flags=p.URDF_USE_SELF_COLLISION,
                            start=[0, 0, PI / 2, 0, 0, 0],)
        self.create_constrain(parentPosition=[-0.1, -0.2, 0.1],childOrientation=[ 0, -0.7068252, 0, 0.7073883 ])

        self.rolling_tool = Gripper(self.id_client)
        self.rolling_tool.load_urdf(fileName="F:\\sw\\urdf_files\\rolling_tool\\urdf\\rolling_tool.urdf",
                                    basePosition=(-1, -1, 1), useFixedBase=0, flags=0)
        self.bind_rolling_tool2robot(self.env.robots[0], self.rolling_tool, pos_in_robot_end_link=[0, 0, 0], )
        self.init_rolling_tool()
        self.init_robot()

        tool_id = self.machine.add_tool_to_machine("F:\\sw\\urdf_files\\tool_6mm.SLDASM\\urdf\\tool_6mm.SLDASM.urdf"
                                                   , tool_length=0.15)
        p.setCollisionFilterPair(self.workpiece_id, tool_id, -1, -1, 0)
        self.nc_code_path = "F:\\UG\\blade\\NC_1.tp"
        self.machine.parse_nc_code(
            # "F:\\python\\python-vtk\\pybulletProject\\cutting_with_rolling\\data\\tp_path.txt",
            # r"F:\sw\滚压\用作机械臂\边切边滚实验方案\ug\6061_simple_精加工-连续.tp",
            # r"F:\python\RobotGUI_2.1\User_Defined_Demos\Robot_Machine\files\机床路径凹面1.tp",
            # r"F:\python\RobotGUI_2.1\User_Defined_Demos\Robot_Machine\files\机床路径凸面1.tp",
            # r"F:\sw\滚压\用作机械臂\边切边滚实验方案\ug\test\测试A轴标准.tp"
            r"F:\sw\滚压\用作机械臂\边切边滚实验方案\ug\test\新pui新刀位_移除机床.tp"
        )

        self.joint_values = self.machine.get_values_with_nc_codes()
        self.joint_values = self.machine.interpolation_path((np.array(self.joint_values)), scale=10,
                                                            id_index=-1)  # 最后一项为编号
        self.robot_goto_positions = self.read_cls_file(r'F:\python\RobotGUI_2.1\User_Defined_Demos\Robot_Machine\files\6061_simple中面刀位.cls',inverse=True)


        # self.debug()

        pass

    def debug(self):

        for i in range(-1, self.env.robots[0].id_end_effector + 1):
            for j in range(-1, self.machine.workpiece.num_all_joints + 1):
                p.setCollisionFilterPair(self.env.robots[0].id_robot, self.workpiece_id, i, j, 0)
            p.setCollisionFilterPair(self.env.robots[0].id_robot, self.machine.id_robot, i, 0, 0)
            p.setCollisionFilterPair(self.env.robots[0].id_robot, self.machine.id_robot, i, -1, 0)
            p.setCollisionFilterPair(self.env.robots[0].id_robot, self.machine.id_robot, i, 1, 0)
            p.setCollisionFilterPair(self.env.robots[0].id_robot, self.machine.id_robot, i, 2, 0)
            p.setCollisionFilterPair(self.env.robots[0].id_robot, self.machine.id_tool, i, -1, 0)
            p.setCollisionFilterPair(self.env.robots[0].id_robot, self.machine.id_tool, i, 0, 0)

        for i in range(-1, self.rolling_tool.num_all_joints + 1):
            for j in range(-1, self.machine.workpiece.num_all_joints + 1):
                p.setCollisionFilterPair(self.rolling_tool.id_robot, self.workpiece_id, i, j, 0)
            p.setCollisionFilterPair(self.rolling_tool.id_robot, self.machine.id_robot, i, 0, 0)
            p.setCollisionFilterPair(self.rolling_tool.id_robot, self.machine.id_robot, i, -1, 0)
            p.setCollisionFilterPair(self.rolling_tool.id_robot, self.machine.id_robot, i, 1, 0)
            p.setCollisionFilterPair(self.rolling_tool.id_robot, self.machine.id_robot, i, 2, 0)
            p.setCollisionFilterPair(self.rolling_tool.id_robot, self.machine.id_tool, i, -1, 0)
            p.setCollisionFilterPair(self.rolling_tool.id_robot, self.machine.id_tool, i, 0, 0)

        for i in range(-1, self.rolling_tool.num_all_joints + 1):
            for j in range(-1, self.env.robots[0].id_end_effector + 1):
                p.setCollisionFilterPair(self.rolling_tool.id_robot, self.env.robots[0].id_robot, i, j, 0)

        # [-0.7068252, 0, 0, 0.7073883] [ 0.0005629, -0.706825, 0.707388, 0.0005633 ],[0.6996540305742056,  0.1023925656542926,  0.1023925656542926,0.6996540305742056]
        robot_target_pos, robot_target_ori = [0, -0.05, 0.08], [ 0.0005629, -0.706825, 0.707388, 0.0005633 ]
        robot_target_pos, robot_target_ori = self.get_point_in_workpiece2robot(robot_target_pos, robot_target_ori)
        robot_target_pos, robot_target_ori = self.env.robots[0].calculate_ee_origin_from_target(robot_target_pos,
                                                                                                robot_target_ori,
                                                                                                self.point_in_ee_frame,
                                                                                                self.robot_target_ori)
        start = [-5.632696646620288, 2.9601766673109164, 1.3607487723362872, 4.320925361170024, -0.9211075418969108, 4.710803686683021]
        joints = self.env.robots[0].calc_path_joints(None, None, robot_target_pos, robot_target_ori, start=start)
        inter_joints = self.env.robots[0].interpolation_path(joints, scale=20, add_more_end=0)
        # # 创建一个持续的生成器实例
        # joint_generator = sys.env.robots[0].run_joints_lists(inter_joints, sys.env.robots[0].id_robot,
        #                                                      sleep_time=1 / 240., KP=0.1, targetVelocity=0)
        for joints in inter_joints:
            for i, joint_position in enumerate(joints):
                # p.resetJointState(self.env.robots[0].id_robot, i, joint_position)

                p.setJointMotorControl2(self.env.robots[0].id_robot, i, p.POSITION_CONTROL,
                                        targetPosition=joint_position,
                                        targetVelocity=0.0,
                                        positionGain=0.5,  # KP
                                        velocityGain=0.8,  # KD
                                        force=1000,
                                        maxVelocity=2,
                                        )
            while not self.env.robots[0].has_reached_target(joints, 1e-3):
                p.stepSimulation()
                time.sleep(1 / 240.)
        print(inter_joints[-1])
        # for _ in range(1000):
        #
        #     p.stepSimulation()
        #     time.sleep(1 / 240)

        pass

    def init_machine(self, type='velocity'):
        self.machine.set_id_end_effector(4)
        init_pos = [0, 0, -0.6, -0.2, 0, 0]
        self.new_joint_ranges = [(None, None), (None, None), (init_pos[2], None), (init_pos[3], None), (None, None),
                                 (None, None)]
        self.machine.set_joint_ranges(self.new_joint_ranges)
        self.machine.set_joints_states(init_pos)
        for i in range(self.machine.num_all_joints):
            p.changeDynamics(
                bodyUniqueId=self.machine.id_robot,
                linkIndex=i,
                jointDamping=0.2,
                mass=10,
                linearDamping=0.5, angularDamping=0.5,
            )
        force = 10000
        if type == 'velocity':
            mode = p.VELOCITY_CONTROL
            p.setJointMotorControl2(self.machine.id_robot, 0, mode, targetVelocity=0, force=force)
            p.setJointMotorControl2(self.machine.id_robot, 1, mode, targetVelocity=0, force=force)
            p.setJointMotorControl2(self.machine.id_robot, 2, mode, targetVelocity=0, force=force)
            p.setJointMotorControl2(self.machine.id_robot, 3, mode, targetVelocity=0, force=force)
            p.setJointMotorControl2(self.machine.id_robot, 4, mode, targetVelocity=0, force=force)
            p.setJointMotorControl2(self.machine.id_robot, 5, mode, targetVelocity=0, force=force)
        else:
            mode = p.POSITION_CONTROL
            p.setJointMotorControl2(self.machine.id_robot, 0, mode, targetPosition=init_pos[0], targetVelocity=0,
                                    force=force)
            p.setJointMotorControl2(self.machine.id_robot, 1, mode, targetPosition=init_pos[1], targetVelocity=0,
                                    force=force)
            p.setJointMotorControl2(self.machine.id_robot, 2, mode, targetPosition=init_pos[2], targetVelocity=0,
                                    force=force)
            p.setJointMotorControl2(self.machine.id_robot, 3, mode, targetPosition=init_pos[3], targetVelocity=0,
                                    force=force)
            p.setJointMotorControl2(self.machine.id_robot, 4, mode, targetPosition=init_pos[4], targetVelocity=0,
                                    force=force)
            p.setJointMotorControl2(self.machine.id_robot, 5, mode, targetPosition=init_pos[5], targetVelocity=0,
                                    force=force)

    def init_robot(self):
        # for i in range(self.env.robots[0].num_avail_joints):
        #     p.resetJointState(self.env.robots[0].id_robot,i,0)
        self.env.robots[0].set_joints_states([0, 0, 0, 0, 0, 0])
        self.env.robots[0].inverse_mode = 'body_sys'
        for i in range(self.env.robots[0].num_all_joints):
            p.changeDynamics(
                bodyUniqueId=self.env.robots[0].id_robot,
                linkIndex=i,
                # jointDamping=0.1,
                mass=0.1,

            )

        for joint in range(6):
            p.changeDynamics(self.env.robots[0].id_robot, joint, linearDamping=0.1, angularDamping=0.5)

        force = 10000
        mode = p.VELOCITY_CONTROL
        p.setJointMotorControl2(self.env.robots[0].id_robot, 0, mode, targetVelocity=0, force=force)
        p.setJointMotorControl2(self.env.robots[0].id_robot, 1, mode, targetVelocity=0, force=force)
        p.setJointMotorControl2(self.env.robots[0].id_robot, 2, mode, targetVelocity=0, force=force)
        p.setJointMotorControl2(self.env.robots[0].id_robot, 3, mode, targetVelocity=0, force=force)
        p.setJointMotorControl2(self.env.robots[0].id_robot, 4, mode, targetVelocity=0, force=force)
        p.setJointMotorControl2(self.env.robots[0].id_robot, 5, mode, targetVelocity=0, force=force)

        # for i in range(self.env.robots[0].id_end_effector):
        #     p.changeDynamics(self.env.robots[0].id_robot, i, mass=10)
        for _ in range(100):
            p.stepSimulation()
            # time.sleep(1/240.)
        pass

    def init_rolling_tool(self):

        self.rolling_tool.set_joints_states([0, 0])
        for i in range(self.rolling_tool.num_all_joints):
            p.changeDynamics(
                bodyUniqueId=self.rolling_tool.id_robot,
                linkIndex=i,
                jointDamping=0.2,
                mass=0.01,
                linearDamping=0.5, angularDamping=0.5,
            )
        force = 100000
        mode = p.POSITION_CONTROL
        p.setJointMotorControl2(self.rolling_tool.id_robot, 0, mode, targetPosition=0, targetVelocity=0,
                                positionGain=0.5,  # KP
                                velocityGain=0.5,  # KD
                                force=force, )
        p.setJointMotorControl2(self.rolling_tool.id_robot, 1, mode, targetPosition=0, targetVelocity=0,
                                positionGain=0.5,  # KP
                                velocityGain=0.5,  # KD
                                force=force, )

        pass

    def bind_rolling_tool2robot(self, robot: Robot, gripper: Gripper, pos_in_robot_end_link=[0, 0, 0],
                                pos_in_gripper_base_link=[0, 0, 0]):

        robot_end_mass = Robot.get_com_in_link_frame(robot.id_robot, robot.id_end_effector)
        robot_end_in_mass_sys = np.array(pos_in_robot_end_link) - np.array(robot_end_mass)

        gripper_mass = Robot.get_com_in_link_frame(gripper.id_robot, -1, baseFramePosition=gripper.baseLinkPosition)
        gripper_in_mass_sys = np.array(pos_in_gripper_base_link) - np.array(gripper_mass)

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

        p.changeConstraint(constraint_id, maxForce=1e8, )
        p.setCollisionFilterPair(gripper.id_robot, robot.id_robot, -1, robot.id_end_effector,
                                 0,
                                 physicsClientId=self.env.id_client)
        self.point_in_ee_frame, self.robot_target_ori = [0, 0, -0.13345], [0, 0, 0, 1]
        for _ in range(100):
            p.stepSimulation()

        pass

    def create_constrain(self, parentPosition=[0, 0, 0], childPosition=[0, 0, 0], childOrientation=[0, 0, 0, 1]):
        # fixed_joint = p.createConstraint(self.env.robots[0].id_robot, 6, self.env.robots[1].id_robot, -1,
        #                                  p.JOINT_FIXED, [0, 0, 1], [0, 0, 0], [0, 0, 0], physicsClientId=self.id_client)
        childOrientation = invert_quaternion(childOrientation)
        fixed_joint = self.env.createConstraint(self.machine.id_robot, 1, self.env.robots[0].id_robot, -1,
                                                p.JOINT_FIXED, [0, 0, 0], parentPosition, childPosition,
                                                childFrameOrientation=childOrientation,
                                                physicsClientId=self.id_client)

        # p.setCollisionFilterPair(self.env.robots[0].id_robot, self.env.robots[1].id_robot, 0, -1, 0,
        #                          physicsClientId=self.id_client)
        # p.setCollisionFilterPair(self.env.robots[0].id_robot, self.env.robots[1].id_robot, 1, -1, 0,
        #                          physicsClientId=self.id_client)

        p.setCollisionFilterPair(self.env.robots[0].id_robot, self.machine.id_robot, -1, 0, 0,
                                 physicsClientId=self.id_client)

        # 记录工件坐标系相对于机械臂本体坐标系的位置和姿态，方便将工件坐标系的点转换到机械臂本体坐标系

        workpiece_pos = self.workpiece_pose
        workpiece_ori = self.workpiece_orientation
        robot_pos = parentPosition
        robot_ori = invert_quaternion(childOrientation)
        # 计算工件坐标系在机械臂坐标系下的表示
        self.T_workpiece2robot = Robot.TAB_with_AinW_and_BinW(workpiece_pos, workpiece_ori, robot_pos, robot_ori)
        # 计算工件坐标系在世界坐标系下的表示 这里均是在C轴坐标系下的表示
        self.T_workpiece2world = Robot.TAB_with_AinW_and_BinW(workpiece_pos, workpiece_ori,[0,0,-0.685],[0,0,0,1])
        # 计算机器人坐标系在世界坐标系下的表示
        self.T_robot2world = Robot.TAB_with_AinW_and_BinW(robot_pos, robot_ori, [0,0,-0.685],[0,0,0,1])
        pass

    def set_R_W_M_collision(self,):
        for i in range(-1, self.env.robots[0].id_end_effector + 1):
            for j in range(-1, self.machine.workpiece.num_all_joints + 1):
                p.setCollisionFilterPair(self.env.robots[0].id_robot, self.workpiece_id, i, j, 0)
            p.setCollisionFilterPair(self.env.robots[0].id_robot, self.machine.id_robot, i, 0, 0)
            p.setCollisionFilterPair(self.env.robots[0].id_robot, self.machine.id_robot, i, -1, 0)
            p.setCollisionFilterPair(self.env.robots[0].id_robot, self.machine.id_robot, i, 1, 0)
            p.setCollisionFilterPair(self.env.robots[0].id_robot, self.machine.id_robot, i, 2, 0)
            p.setCollisionFilterPair(self.env.robots[0].id_robot, self.machine.id_tool, i, -1, 0)
            p.setCollisionFilterPair(self.env.robots[0].id_robot, self.machine.id_tool, i, 0, 0)

        for i in range(-1, self.rolling_tool.num_all_joints + 1):
            for j in range(-1, self.machine.workpiece.num_all_joints + 1):
                p.setCollisionFilterPair(self.rolling_tool.id_robot, self.workpiece_id, i, j, 0)
            p.setCollisionFilterPair(self.rolling_tool.id_robot, self.machine.id_robot, i, 0, 0)
            p.setCollisionFilterPair(self.rolling_tool.id_robot, self.machine.id_robot, i, -1, 0)
            p.setCollisionFilterPair(self.rolling_tool.id_robot, self.machine.id_robot, i, 1, 0)
            p.setCollisionFilterPair(self.rolling_tool.id_robot, self.machine.id_robot, i, 2, 0)
            p.setCollisionFilterPair(self.rolling_tool.id_robot, self.machine.id_tool, i, -1, 0)
            p.setCollisionFilterPair(self.rolling_tool.id_robot, self.machine.id_tool, i, 0, 0)

        for i in range(-1, self.rolling_tool.num_all_joints + 1):
            for j in range(-1, self.env.robots[0].id_end_effector + 1):
                p.setCollisionFilterPair(self.rolling_tool.id_robot, self.env.robots[0].id_robot, i, j, 0)


    def get_point_in_workpiece2robot(self, pos, ori,inverse=False):
        if self.T_workpiece2robot is not None:
            if inverse==False:
                rot = R.from_quat(ori).as_matrix()
                T_p = np.eye(4)
                T_p[:3, :3] = rot
                T_p[:3, 3] = pos
                trans_T = np.dot(self.T_workpiece2robot, T_p)
                trans_pos, trans_ori = trans_T[:3, 3], R.from_matrix(trans_T[:3, :3]).as_quat()
            else:
                rot = R.from_quat(ori).as_matrix()
                T_p = np.eye(4)
                T_p[:3, :3] = rot
                T_p[:3, 3] = pos
                trans_T = np.dot(np.linalg.inv(self.T_workpiece2robot), T_p)
                trans_pos, trans_ori = trans_T[:3, 3], R.from_matrix(trans_T[:3, :3]).as_quat()
            return trans_pos, trans_ori
        else:
            print("机械臂和工件的约束关系未建立")
        pass
    def get_point_in_workpiece2world(self, pos, ori,inverse=False):
        if self.T_workpiece2world is not None:
            if inverse==False:
                rot = R.from_quat(ori).as_matrix()
                T_p = np.eye(4)
                T_p[:3, :3] = rot
                T_p[:3, 3] = pos
                trans_T = np.dot(self.T_workpiece2world, T_p)
                trans_pos, trans_ori = trans_T[:3, 3], R.from_matrix(trans_T[:3, :3]).as_quat()
                return trans_pos, trans_ori
            else:
                rot = R.from_quat(ori).as_matrix()
                T_p = np.eye(4)
                T_p[:3, :3] = rot
                T_p[:3, 3] = pos
                trans_T = np.dot(np.linalg.inv(self.T_workpiece2world), T_p)
                trans_pos, trans_ori = trans_T[:3, 3], R.from_matrix(trans_T[:3, :3]).as_quat()
                return trans_pos, trans_ori
        else:
            print("机械臂和工件的约束关系未建立")
        pass

    def get_work_piece_sys_point_in_world_sys(self, pos=[], ori=[]):
        workpiece_state = p.getLinkState(self.workpiece_id, 0)
        workpiece_pos = np.array(workpiece_state[4])
        workpiece_ori = np.array(workpiece_state[5])

        # 计算旋转矩阵
        rotation_matrix = np.array(p.getMatrixFromQuaternion(workpiece_ori)).reshape(3, 3)

        # 构建齐次变换矩阵
        transform_matrix = np.eye(4)
        transform_matrix[:3, :3] = rotation_matrix
        transform_matrix[:3, 3] = workpiece_pos

        world_points = []
        world_orientations = []

        for point, orientation in zip(pos, ori):
            # 将点转换为齐次坐标
            local_point_homogeneous = np.append(point, 1)

            # 进行坐标变换
            world_point_homogeneous = np.dot(transform_matrix, local_point_homogeneous)

            # 提取转换后的点 (x, y, z)
            world_points.append(world_point_homogeneous[:3])

            # 将局部姿态转换为世界姿态
            local_rotation = R.from_quat(orientation)
            world_rotation = R.from_matrix(rotation_matrix) * local_rotation
            world_orientations.append(world_rotation.as_quat())

        return world_points, world_orientations

        pass

    def hold_robot_state(self, joint_list: list):
        force = 100
        mode = p.POSITION_CONTROL
        for i, joint in enumerate(joint_list):
            p.setJointMotorControl2(self.env.robots[0].id_robot, i, mode, targetPosition=joint, targetVelocity=0,
                                    positionGain=0.05,  # KP
                                    velocityGain=0.2,  # KD
                                    force=force
                                    )
        pass

    def read_cls_file(self,cls_file_path,inverse=False):
        """
        读取CLS文件并将其转换为字典格式，同时计算四元数。

        :param cls_file_path: CLS文件路径
        :return: 包含位置和四元数的字典列表
        """
        with open(cls_file_path, 'r') as file:
            lines = file.readlines()

        pattern = re.compile(r'GOTO/([\d\.\-]+),([\d\.\-]+),([\d\.\-]+),?([\d\.\-]*)?,?([\d\.\-]*)?,?([\d\.\-]*)?')

        result = []

        for line in lines:
            match = pattern.match(line.strip())
            if match:
                x = float(match.group(1))
                y = float(match.group(2))
                z = float(match.group(3))
                i = float(match.group(4)) if match.group(4) else 1.0
                j = float(match.group(5)) if match.group(5) else 0.0
                k = float(match.group(6)) if match.group(6) else 0.0

                # 计算四元数
                quat = self.calculate_quaternion(i, j, k,inverse=inverse)
                pos,ori = self.get_point_in_workpiece2robot([x/1000,y/1000,z/1000], [quat[1],quat[2],quat[3],quat[0],])
                pos,ori = self.env.robots[0].calculate_ee_origin_from_target(pos, ori,
                                                                   self.point_in_ee_frame,
                                                                   self.robot_target_ori)
                result.append({
                    'X': pos[0],
                    'Y': pos[1],
                    'Z': pos[2],
                    'O': {'x': ori[0], 'y': ori[1], 'z': ori[2],'w': ori[3], }
                })

        return result
    @staticmethod
    def calculate_quaternion(i, j, k,inverse=False):
        """
        根据给定的方向向量计算四元数，使其x轴对齐方向向量，y轴对齐世界坐标系的Z轴。

        :param i: x方向分量
        :param j: y方向分量
        :param k: z方向分量
        :return: 对应的四元数
        """
        # 创建方向向量
        direction = np.array([i, j, k])
        direction_norm = np.linalg.norm(direction)
        if direction_norm == 0:
            raise ValueError("The direction vector cannot be zero.")
        direction = direction / direction_norm

        old_z = np.array([0, 0, 1])

        if inverse:
            direction = -direction
            old_z = -old_z

        # 确定新的坐标系的z轴（与世界坐标系Z轴对齐）
        new_y_axis = old_z

        # 确定新的坐标系的x轴（与方向向量对齐）
        new_x_axis = direction

        # 计算新的坐标系的y轴（新x轴与新z轴的叉积）
        new_z_axis = np.cross(new_x_axis,new_y_axis)
        new_z_axis = new_z_axis / np.linalg.norm(new_z_axis)

        # # 重新计算新的x轴（确保正交性）
        # new_x_axis = np.cross(new_y_axis, new_z_axis)
        # new_x_axis = new_x_axis / np.linalg.norm(new_x_axis)

        # 构建旋转矩阵
        rotation_matrix = np.array([new_x_axis, new_y_axis, new_z_axis]).T

        # 计算四元数
        quat = Quaternion(matrix=rotation_matrix)
        return quat

    def convert_positions_to_joints(self,robot_goto_positions=None ,start=None,save=False,filename="joints_list.npy"):
        """
        读取sys.robot_goto_positions中的位置和四元数，调用calc_path_joints函数，将其转换为关节列表。

        :param sys: 系统对象，包含机器人和环境信息
        :param start: 初始位置（可选）
        :return: 包含关节列表的列表
        """
        joints_list = []
        if robot_goto_positions is None:
            robot_goto_positions = self.robot_goto_positions


        for item in tqdm(robot_goto_positions,desc="convert_positions_to_joints:计算机械臂刀位点路径"):
            # 提取位置和四元数
            robot_target_pos = [item['X'], item['Y'], item['Z']]
            robot_target_ori = [item['O']['x'], item['O']['y'], item['O']['z'],item['O']['w']]
            # 为什么会多出来0？
            # 计算路径关节
            joints = self.env.robots[0].get_state_from_ik(robot_target_pos,robot_target_ori,start=start)
            start = list(copy.deepcopy(joints))
            joints_list.append((joints))
        print("计算完成,共计", len(joints_list), "个点。")
        if save:
            np.save(f"F:\\python\\RobotGUI_2.1\\User_Defined_Demos\\Robot_Machine\\files\\{filename}", joints_list)

        return joints_list



    def run(self):
        """ Main loop to run the simulation. """
        # command_thread = threading.Thread(target=self.controller.execute_commands, args=(self,))
        # command_thread.start()
        while True:
            self.controller.execute_commands(self)
            if self.controller.finished:
                break
            self.sys_update()
    def sys_update(self):
        self.env.update(mode=1, robot_mode=0)
        # self.camera.show_link_sys(-1, lifetime=0.1,type=1)
        # self.env.robots[0].show_link_sys(self.env.robots[0].id_end_effector,lifetime=0.1,type=1)
        time.sleep(self.step_time)
        pass

    def env_hold_on(self):

        while True:
            # self.machine.update_machine_state(self.joint_values, mode="Reset0",time_scale=1,tolerance=1e-3)
            self.machine.update_machine_state(self.joint_values, mode="Reset0", time_scale=0.005, tolerance=1e-3,
                                              xyzVelocity=0.5, acVelocity=2)
            # self.machine.execute_nc_code(self.nc_code_path)
            self.env.update(mode=1, robot_mode=2)
            time.sleep(1 / 240.)


if __name__ == '__main__':
    demo = RM_sys(True)
    # demo.env_hold_on()
    goto = demo.read_cls_file(r'F:\python\RobotGUI_2.1\User_Defined_Demos\Robot_Machine\files\滚压刀位点.cls',inverse=True)
    demo.convert_positions_to_joints(goto,save=True,filename="Aubo_i3_standard.npy",start=[-4.768899762752129, 2.5471347307044185, 1.5917968035676557, 4.139163680609113, -0.056207696337930955, 4.71112355916304])
    while True:
        p.stepSimulation()




