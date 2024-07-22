import time
import pybullet as p
from utils import *
from Environment.PybulletEnv import PybulletEnv
from RobotDir.Robot import Robot
from RobotDir.Machine import Machine
from RobotDir.Gripper import Gripper
from scipy.spatial.transform import Rotation as R
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ''))
from RobotDir.utils.my_utils import invert_quaternion

class BaseDemo:
    # 创建pybullet连接
    def __init__(self):
        self.id_client = p.connect(p.GUI)
        p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1, shadowMapWorldSize=1, shadowMapIntensity=1, physicsClientId=self.id_client)
        p.setAdditionalSearchPath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '../RobotDir/urdf/'))
        self.default_init()
        
        
    def default_init(self):
        self.env = PybulletEnv(id_client=self.id_client)
        workpiece_urdf = r"F:\sw\urdf_files\6061_simple\urdf\6061_simple.urdf"
        machine_file_name = "F:\sw\\urdf_files\c501-simple.SLDASM\\urdf\c501-simple.SLDASM.urdf"
        # robot_urdf = "F:\sw\\urdf_files\i7-nofork.SLDASM\\urdf\i7-nofork.SLDASM.urdf"
        robot_urdf = "F:\sw\\urdf_files\Aubo_i3\\urdf\Aubo_i3.urdf"
        # robot_urdf = r"F:\sw\\urdf_files\UR3\\urdf\UR3.urdf"
        self.machine = Machine(self.id_client)
        self.machine.load_urdf(fileName=machine_file_name, basePosition=(0, 0, 0), useFixedBase=1, flags=0,)
        self.init_machine(type='p')
        self.workpiece_pose = [0.0, 0.2, 0.02]
        orientation= [0.0, 0.0, 0]
        quaternion = p.getQuaternionFromEuler(orientation)
        self.workpiece_orientation = quaternion
        self.workpiece_id = self.machine.add_workpiece_to_machine(workpiece_urdf, position=self.workpiece_pose,orientation=orientation)

        self.env.load_robot(fileName=robot_urdf, basePosition=(-0.15, -0.15, 0.7), useFixedBase=0,start=[0, 0, PI / 2, 0, 0, 0],
                            # flags=p.URDF_USE_SELF_COLLISION,
                            )
        self.create_constrain(parentPosition=[-0.1, -0.1, 0.1],childOrientation=[ 0, -0.7068252, 0, 0.7073883 ])

        self.rolling_tool = Gripper(self.id_client)
        self.rolling_tool.load_urdf(fileName="F:\\sw\\urdf_files\\rolling_tool\\urdf\\rolling_tool.urdf",
                                    basePosition=(-1, -1, 1), useFixedBase=0, flags=0)
        self.bind_rolling_tool2robot(self.env.robots[0], self.rolling_tool, pos_in_robot_end_link=[0, 0, 0],)
        self.init_rolling_tool()
        self.init_robot('position')



        tool_id = self.machine.add_tool_to_machine("F:\\sw\\urdf_files\\tool_6mm.SLDASM\\urdf\\tool_6mm.SLDASM.urdf"
                                                 , tool_length=0.15)
        p.setCollisionFilterPair(self.workpiece_id, tool_id, -1, -1, 0)
        self.nc_code_path = "F:\\UG\\blade\\NC_1.tp"
        self.machine.parse_nc_code(
            # "F:\\python\\python-vtk\\pybulletProject\\cutting_with_rolling\\data\\tp_path.txt",
             r"F:\sw\滚压\用作机械臂\边切边滚实验方案\ug\6061_simple_精加工-连续.tp",
        )

        self.joint_values = self.machine.get_values_with_nc_codes()
        self.joint_values = self.machine.interpolation_path((np.array(self.joint_values)),scale=10,id_index=-1) # 最后一项为编号

        self.debug()

        pass


    def debug(self):



        for i in range(-1,self.env.robots[0].id_end_effector+1):
            for j in range(-1,self.machine.workpiece.num_all_joints+1):
                p.setCollisionFilterPair(self.env.robots[0].id_robot, self.workpiece_id, i, j, 0)
            p.setCollisionFilterPair(self.env.robots[0].id_robot, self.machine.id_robot, i, 0, 0)
            p.setCollisionFilterPair(self.env.robots[0].id_robot, self.machine.id_robot, i, -1, 0)
            p.setCollisionFilterPair(self.env.robots[0].id_robot, self.machine.id_robot, i, 1, 0)
            p.setCollisionFilterPair(self.env.robots[0].id_robot, self.machine.id_robot, i, 2, 0)
            p.setCollisionFilterPair(self.env.robots[0].id_robot, self.machine.id_tool, i, -1, 0)
            p.setCollisionFilterPair(self.env.robots[0].id_robot, self.machine.id_tool, i, 0, 0)

        for i in range(-1,self.rolling_tool.num_all_joints+1):
            for j in range(-1,self.machine.workpiece.num_all_joints+1):
                p.setCollisionFilterPair(self.rolling_tool.id_robot, self.workpiece_id, i, j, 0)
            p.setCollisionFilterPair(self.rolling_tool.id_robot, self.machine.id_robot, i, 0, 0)
            p.setCollisionFilterPair(self.rolling_tool.id_robot, self.machine.id_robot, i, -1, 0)
            p.setCollisionFilterPair(self.rolling_tool.id_robot, self.machine.id_robot, i, 1, 0)
            p.setCollisionFilterPair(self.rolling_tool.id_robot, self.machine.id_robot, i, 2, 0)
            p.setCollisionFilterPair(self.rolling_tool.id_robot, self.machine.id_tool, i, -1, 0)
            p.setCollisionFilterPair(self.rolling_tool.id_robot, self.machine.id_tool, i, 0, 0)

        for i in range(-1,self.rolling_tool.num_all_joints+1):
            for j in range(-1,self.env.robots[0].id_end_effector+1):
                p.setCollisionFilterPair(self.rolling_tool.id_robot,self.env.robots[0].id_robot, i, j, 0)

        # [-0.7068252, 0, 0, 0.7073883]
        robot_target_pos, robot_target_ori = [0, -0.05, 0.08], [ 0.0005629, -0.706825, 0.707388, 0.0005633 ]
        robot_target_pos, robot_target_ori = self.get_point_in_workpiece2robot(robot_target_pos, robot_target_ori)
        robot_target_pos, robot_target_ori = self.env.robots[0].calculate_ee_origin_from_target(robot_target_pos, robot_target_ori, self.point_in_ee_frame,
                                                                              self.robot_target_ori)
        # start = [-0.76780261, -0.29909405, -2.44598745,  0.39766279,  2.37378243, -3.14235681]
        start = None
        start = [0.3371707692333656, +2.2, 1.5621066164537876, 2.847458386659798-3.14, 1.570539796590778-3.14, -0.33875969082208246+3.14]
        start = [0.3371707692333656, +2.2, 1.5621066164537876, 2.847458386659798-3.14, 1.570539796590778, -0.33875969082208246]
        start = [2.8768414067774057-3.14, -1.2866527131781031, 2.7759853735326994, -1.6553270137037546, -2.8745074124611616, -1.5753631584999404]
        start = [-3.4275858791860574, 2.0309533882895305, 1.6820382332838875, -1.2371244849223977, -0.8959838675420609, 3.511014803657219]
        # start = [-6.0145556627693635+3.14-0.5, 3.9336018762764637-1.4, 4.659111545186258-1.5-1.57, -3.970679790611662, -2.873810105155704+3.14, 4.713653201640938]
        start = [-3.7920836753938008, 0.19025516272629195, 2.379552161482645, -0.5717854079303571, -0.9210978342525463, 4.710803686732424]  # 躺着
        start = [-5.632696646620288, 2.9601766673109164, 1.3607487723362872, 4.320925361170024, -0.9211075418969108, 4.710803686683021] # 躺着

        joints = self.env.robots[0].calc_path_joints(None, None, robot_target_pos, robot_target_ori, start=start)
        inter_joints = self.env.robots[0].interpolation_path(joints, scale=20, add_more_end=0)
        # # 创建一个持续的生成器实例
        # joint_generator = sys.env.robots[0].run_joints_lists(inter_joints, sys.env.robots[0].id_robot,
        #                                                      sleep_time=1 / 240., KP=0.1, targetVelocity=0)

        print('机械臂目标关机角', inter_joints[-1])
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

        # for _ in range(1000):
        #
        #     p.stepSimulation()
        #     time.sleep(1 / 240)

        pass


    def init_machine(self,type='velocity'):
        self.machine.set_id_end_effector(4)
        init_pos = [0, 0, -0.6, -0.2, 0, 0]
        self.new_joint_ranges = [(None, None), (None, None), (init_pos[2], None), (init_pos[3], None), (None, None),(None, None) ]
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
        if type=='velocity':
            mode = p.VELOCITY_CONTROL
            p.setJointMotorControl2(self.machine.id_robot, 0, mode, targetVelocity=0, force=force)
            p.setJointMotorControl2(self.machine.id_robot, 1, mode, targetVelocity=0, force=force)
            p.setJointMotorControl2(self.machine.id_robot, 2, mode, targetVelocity=0, force=force)
            p.setJointMotorControl2(self.machine.id_robot, 3, mode, targetVelocity=0, force=force)
            p.setJointMotorControl2(self.machine.id_robot, 4, mode, targetVelocity=0, force=force)
            p.setJointMotorControl2(self.machine.id_robot, 5, mode, targetVelocity=0, force=force)
        else:
            mode= p.POSITION_CONTROL
            p.setJointMotorControl2(self.machine.id_robot, 0, mode, targetPosition=init_pos[0], targetVelocity=0,force=force)
            p.setJointMotorControl2(self.machine.id_robot, 1, mode, targetPosition=init_pos[1], targetVelocity=0,force=force)
            p.setJointMotorControl2(self.machine.id_robot, 2, mode, targetPosition=init_pos[2], targetVelocity=0,force=force)
            p.setJointMotorControl2(self.machine.id_robot, 3, mode, targetPosition=init_pos[3], targetVelocity=0,force=force)
            p.setJointMotorControl2(self.machine.id_robot, 4, mode, targetPosition=init_pos[4], targetVelocity=0,force=force)
            p.setJointMotorControl2(self.machine.id_robot, 5, mode, targetPosition=init_pos[5], targetVelocity=0,force=force)

    def init_robot(self,type='velocity'):
        # for i in range(self.env.robots[0].num_avail_joints):
        #     p.resetJointState(self.env.robots[0].id_robot,i,0)
        init_pos = [0, 0, 0, 0, 0, 0]
        self.env.robots[0].set_joints_states(init_pos)
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
        if type == 'velocity':
            mode = p.VELOCITY_CONTROL
            p.setJointMotorControl2(self.env.robots[0].id_robot, 0, mode, targetVelocity=0, force=force)
            p.setJointMotorControl2(self.env.robots[0].id_robot, 1, mode, targetVelocity=0, force=force)
            p.setJointMotorControl2(self.env.robots[0].id_robot, 2, mode, targetVelocity=0, force=force)
            p.setJointMotorControl2(self.env.robots[0].id_robot, 3, mode, targetVelocity=0, force=force)
            p.setJointMotorControl2(self.env.robots[0].id_robot, 4, mode, targetVelocity=0, force=force)
            p.setJointMotorControl2(self.env.robots[0].id_robot, 5, mode, targetVelocity=0, force=force)
        else:
            mode = p.POSITION_CONTROL
            p.setJointMotorControl2(self.env.robots[0].id_robot, 0, mode, targetPosition=init_pos[0], targetVelocity=0,
                                    force=force)
            p.setJointMotorControl2(self.env.robots[0].id_robot, 1, mode, targetPosition=init_pos[1], targetVelocity=0,
                                    force=force)
            p.setJointMotorControl2(self.env.robots[0].id_robot, 2, mode, targetPosition=init_pos[2], targetVelocity=0,
                                    force=force)
            p.setJointMotorControl2(self.env.robots[0].id_robot, 3, mode, targetPosition=init_pos[3], targetVelocity=0,
                                    force=force)
            p.setJointMotorControl2(self.env.robots[0].id_robot, 4, mode, targetPosition=init_pos[4], targetVelocity=0,
                                    force=force)
            p.setJointMotorControl2(self.env.robots[0].id_robot, 5, mode, targetPosition=init_pos[5], targetVelocity=0,
                                    force=force)

        # for i in range(self.env.robots[0].id_end_effector):
        #     p.changeDynamics(self.env.robots[0].id_robot, i, mass=10)
        for _ in range(10000):
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
                                                force=force,)
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
        self.point_in_ee_frame, self.robot_target_ori = [0,0,-0.13345], [0,0,0,1]
        for _ in range(100):
            p.stepSimulation()


        pass


    def create_constrain(self,parentPosition=[0,0,0],childPosition=[0,0,0],childOrientation=[0,0,0,1]):
        # fixed_joint = p.createConstraint(self.env.robots[0].id_robot, 6, self.env.robots[1].id_robot, -1,
        #                                  p.JOINT_FIXED, [0, 0, 1], [0, 0, 0], [0, 0, 0], physicsClientId=self.id_client)

        childOrientation = invert_quaternion(childOrientation)
        fixed_joint = self.env.createConstraint(self.machine.id_robot, 1, self.env.robots[0].id_robot, -1,
                                                p.JOINT_FIXED, [0, 0, 0], parentPosition, childPosition,childFrameOrientation=childOrientation,
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
        self.T_workpiece2robot = Robot.TAB_with_AinW_and_BinW(workpiece_pos,workpiece_ori,robot_pos,robot_ori)


        pass

    def get_point_in_workpiece2robot(self,pos,ori):
        if self.T_workpiece2robot is not None:
            rot = R.from_quat(ori).as_matrix()
            T_p = np.eye(4)
            T_p[:3, :3] = rot
            T_p[:3, 3] = pos
            trans_T = np.dot(self.T_workpiece2robot, T_p)
            trans_pos,trans_ori = trans_T[:3,3],R.from_matrix(trans_T[:3,:3]).as_quat()
            return trans_pos,trans_ori
        else:
            print("机械臂和工件的约束关系未建立")
        pass


    def get_work_piece_sys_point_in_world_sys(self,pos=[],ori=[]):
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


    def hold_robot_state(self,joint_list:list):
        force = 100
        mode = p.POSITION_CONTROL
        for i,joint in enumerate(joint_list):
            p.setJointMotorControl2(self.env.robots[0].id_robot,i,  mode, targetPosition=joint,targetVelocity=0,
                                    positionGain=0.05,  # KP
                                    velocityGain=0.2,  # KD
                                    force=force
                                    )
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
    demo = BaseDemo()
    demo.env_hold_on()



