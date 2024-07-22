import time
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ''))
import numpy as np
import pybullet as p
import sys
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ''))
import functools
import pickle
from utils import *
from Environment.PybulletEnv import PybulletEnv
from RobotDir.Robot import Robot
from RobotDir.Machine import Machine
from RobotDir.Gripper import Gripper
import pybullet_data
from RobotDir.Camera import Camera
from RobotDir.utils.my_utils import *
import threading


def record_time(func):
    """Decorator to record the execution time of a function."""

    @functools.wraps(func)
    def wrapper(*args, **kwargs):
        start_time = time.time()
        result = func(*args, **kwargs)
        end_time = time.time()
        elapsed_time = end_time - start_time
        print(f"{func.__name__} executed in {elapsed_time:.4f} seconds.")
        return result

    return wrapper

class Command:

    _id_counter = 0  # 类变量，用于自动生成命令ID
    def __init__(self, type, parallel=False,id=None,parallel_id=0,wait_time=0,Command_ifshow=False,**kwargs):
        self.type = type
        self.data = kwargs
        self.parallel = parallel
        self.wait_time = wait_time
        self.generator = None
        self.completed = False
        self.id = id if id is not None else Command._id_counter
        self.parallel_id = parallel_id
        Command._id_counter += 1
        self.start_time = None  # 开始时间
        self.end_time = None  # 结束时间
        self.Command_ifshow = Command_ifshow


    def execute(self, sys):
        """ Execute the command within the given environment as a coroutine. """
        if not self.generator:
            self.start_time = time.time()  # 记录开始时间
            # Decide which generator to use based on command type
            if self.type == "move_arm":
                self.generator = self.move_arm(sys, **self.data)
            elif self.type == "close_gripper":
                self.generator = self.close_gripper(sys)
            elif self.type == "open_gripper":
                self.generator = self.open_gripper(sys)
            elif self.type == 'wait':
                self.generator = self.wait(**self.data)
            elif self.type == 'move_by_delta':
                self.generator = self.move_by_delta(sys,**self.data)
            elif self.type == 'reset':
                self.generator = self.reset(sys,**self.data)
            elif self.type == 'terminate':
                self.generator = self.terminate(sys,)
            elif self.type == 'get_end_effector_data':
                self.generator = self.get_end_effector_data(sys, )
            elif self.type == 'get_point_cloud':
                self.generator = self.get_point_cloud(sys, **self.data)
            elif self.type == 'add_debug_sys':
                self.generator = self.add_debug_sys(sys, **self.data)
            elif self.type == 'show_link_state':
                self.generator = self.show_link_state(sys, **self.data)


        try:
            next(self.generator)
        except StopIteration:
            self.completed = True
            self.end_time = time.time()  # 记录结束时间
            self.print_execution_time()  # 打印执行时间

    def print_execution_time(self):
        """ Print the execution time of the command. """
        if self.start_time and self.end_time:
            elapsed_time = self.end_time - self.start_time
            if self.Command_ifshow:
                print(f"Command {self.type} ID {self.id} executed in {elapsed_time:.4f} seconds.")


    def terminate(self,sys):
        p.disconnect(physicsClientId=sys.env.id_client)
        sys.controller.finished = True
        yield
        pass

    def reset(self,sys, **data):

        sys.controller.stop_current_command()
        sys.controller.command_queue = []
        # sys.reset_sys()
        sys.gp.reset_joint_states()
        sys.gp.open('usePosition', targetPosition=0.0, force=100)

        origin_pos = data.get('origin_pos',[-0.5,-0.3,0.2])
        origin_ori = data.get('origin_ori',[0,0,0,1])
        box_pos = data.get('box_pos',[-0.2, -0.4, 0])
        box_ori = data.get('box_ori',[0,0,0,1])
        target_position = data.get('target_position',[-0.5, -0.3, 0.2])

        p.resetBasePositionAndOrientation(sys.gp.id_robot, origin_pos,origin_ori)
        p.resetBasePositionAndOrientation(sys.camera.id_robot, origin_pos, origin_ori)

        origin_joints = list(sys.env.robots[0].get_state_from_ik(origin_pos, origin_ori))
        p.resetBasePositionAndOrientation(sys.box_id, box_pos,box_ori)
        for i, joint_position in enumerate(origin_joints):
            p.resetJointState(sys.env.robots[0].id_robot, i,origin_joints[i],physicsClientId=sys.env.id_client)
        for i, joint_position in enumerate(origin_joints):
            p.setJointMotorControl2(sys.env.robots[0].id_robot, i, p.POSITION_CONTROL,
                                    targetPosition=joint_position,
                                    targetVelocity=0.0,
                                    positionGain=0.1,  # KP
                                    velocityGain=0.3,  # KD
                                    force=1000,
                                    )
        p.addUserDebugPoints([target_position], [[1, 0, 0]], pointSize=10)

        yield
        pass


    def get_end_effector_data(self,sys):
        pos, ori = sys.env.robots[0].get_end_effector_info()
        data = (pos,ori)
        sys.data_queue.put(data)
        yield
        pass

    def move_by_delta(self,sys, **data):
        delta_pos = data.get('delta_pos',[0.1,0,0])
        delta_ori = data.get('delta_ori',[0,0,0,1])
        scale = data.get('scale', 20)
        KP = data.get('KP', 0.3)
        KD = data.get('KD', 0.5)
        pos,ori = sys.env.robots[0].get_end_effector_info()
        if not delta_ori:
            tar_ori = ori
        else:
            tar_ori = np.array(ori) + np.array(delta_ori)
        tar_pos = np.array(pos) + np.array(delta_pos)
        joints = sys.env.robots[0].calc_path_joints(None, None, tar_pos, tar_ori)
        inter_joints = sys.env.robots[0].interpolation_path(joints, scale=scale)
        # # 创建一个持续的生成器实例
        # joint_generator = sys.env.robots[0].run_joints_lists(inter_joints, sys.env.robots[0].id_robot,
        #                                                      sleep_time=1 / 240., KP=0.1, targetVelocity=0)

        for joints in inter_joints:
            for i, joint_position in enumerate(joints):
                p.setJointMotorControl2(sys.env.robots[0].id_robot, i, p.POSITION_CONTROL,
                                        targetPosition=joint_position,
                                        targetVelocity=0.0,
                                        positionGain=KP,  # KP
                                        velocityGain=KD,  # KD
                                        force=1000,
                                        )

            yield 0
        return 1

        pass

    def move_arm(self, sys, **data):
        """ A generator to simulate the movement of an arm over multiple simulation steps. """
        # Simulate a gradual movement by breaking it into steps
        positions = data.get('positions',[[0,0,0],[0,0,0,1]])
        scale = data.get('scale', 30)
        KP = data.get('KP', 0.1)
        KD = data.get('KD', 0.2)
        point_in_ee_frame = data.get('point_in_ee_frame',None)
        start = data.get('start',None)
        control_type = data.get('control_type','position')
        # pos_in_ee_frame,ori_in_ee_frame=sys.env.robots[0].get_position_relative_to_link(sys.camera.id_robot,sys.env.robots[0].id_robot, 3,sys.env.robots[0].id_end_effector,  )

        robot_target_pos, robot_target_ori = positions[0],positions[1]
        if point_in_ee_frame:
            robot_target_pos = sys.env.robots[0].calculate_ee_origin_from_target(robot_target_pos, point_in_ee_frame,  robot_target_ori)

        joints = sys.env.robots[0].calc_path_joints(None, None, robot_target_pos, robot_target_ori,start=start)
        inter_joints = sys.env.robots[0].interpolation_path(joints, scale=scale,add_more_end=2)
        # # 创建一个持续的生成器实例
        # joint_generator = sys.env.robots[0].run_joints_lists(inter_joints, sys.env.robots[0].id_robot,
        #                                                      sleep_time=1 / 240., KP=0.1, targetVelocity=0)

        if control_type=='position':
            for joints in inter_joints:
                for i, joint_position in enumerate(joints):
                    p.setJointMotorControl2( sys.env.robots[0].id_robot, i, p.POSITION_CONTROL, targetPosition=joint_position,
                                            targetVelocity=0.0,
                                            positionGain=KP,  # KP
                                            velocityGain=KD,  # KD
                                            force=1000,
                                            )
                yield 0
        else:
            for i, joint_position in enumerate(joints[-1]):
                p.resetJointState(sys.env.robots[0].id_robot, i, joint_position, physicsClientId=sys.env.id_client)
                p.stepSimulation()
            for _ in range(100):
                p.stepSimulation()
            for i, joint_position in enumerate(joints[-1]):
                p.resetJointState(sys.env.robots[0].id_robot, i, joint_position, physicsClientId=sys.env.id_client)
            for _ in range(100):
                p.stepSimulation()
            yield 0


        return 1

        # while True:
        #     try:
        #         completed = next(joint_generator)
        #     except StopIteration:
        #         completed = True
        #
        #     if completed:
        #         break
        #     yield  # Yield control and wait for the next call

    def close_gripper(self, sys):
        """ A generator for closing the gripper, simulated over multiple steps. """

        sys.gp.close('useVelocity', targetVelocity=0.3, force=100)
        yield  # Simulate the gripper takes time to close
        # yield  # Continue simulating if needed

    def open_gripper(self, sys):
        """ A generator for closing the gripper, simulated over multiple steps. """

        sys.gp.open('useVelocity', targetVelocity=0.3, force=100)
        yield

    def wait(self,**data):
        sleep_time = data.get('sleep_time',5)
        start_time = time.time()
        while True:
            current_time = time.time()
            if current_time-start_time>=sleep_time:
                break
            yield
        pass


    def get_point_cloud(self,sys,**data):

        ifshow = data.get('ifshow', False)

        depth, rgb = sys.RGB_camera.get_true_depth(sys.RGB_camera.latest_img_data), sys.RGB_camera.get_RGB_img(sys.RGB_camera.latest_img_data)
        body_pcd = sys.RGB_camera.depth_to_pointcloud(depth, rgb)
        pos_ori = p.getLinkState(sys.camera.id_robot, 3)
        pos, ori = pos_ori[4], pos_ori[5]
        TF = sys.RGB_camera.create_transformation_matrix(pos, ori)
        world_pcd = sys.RGB_camera.transform_point_cloud(body_pcd, TF)
        if ifshow:
            sys.RGB_camera.show_pcd_with_orin([world_pcd])

        points = np.asarray(world_pcd.points)
        colors = np.asarray(world_pcd.colors)

        # 将点云数据放入队列中 转化为numpy对象才可以传递
        # 后续可改为共享内存
        sys.data_queue.put(pickle.dumps((points, colors), protocol=pickle.HIGHEST_PROTOCOL))

        yield
        pass

    def add_debug_sys(self,sys,**data):
        pos = data.get('pos',[0,0,0])
        ori = data.get('ori',[0,0,0,1])
        lifetime = data.get('lifetime',10)
        add_debug_sys(p,pos,ori,lifetime,physicsClientId=sys.env.id_client)
        yield
        pass

    def show_link_state(self,sys,**data):

        sys.env.robots[0].show_link_sys(linkIndex=sys.env.robots[0].id_end_effector, lifetime=0, type=1)
        yield
        pass


class Controller:

    def __init__(self):
        self.command_queue = []
        self.active_commands = []
        self.this_command = Command('None',)
        self.this_command.completed = True
        self.finished = False


    def add_command(self, command):
        if command.parallel:
            self.active_commands.append(command)
        else:
            self.command_queue.append(command)

    def stop_current_command(self):
        if not self.this_command.completed:
            print(f"Stopping current command: {self.this_command.type}, id: {self.this_command.id}")
            # Implement any logic to stop the current command here
            # For example, if the current command is a generator, you could interrupt it somehow
            # For simplicity, let's mark it as completed
            self.this_command.completed = True

    def execute_commands(self, env):


        # Process the queue if no non-parallel commands are active
        # if not self.active_commands:
        if self.command_queue and self.this_command.completed:
            ##############################
            """
            为什么self.get_state(reset_command.id)不能再reset后执行
            · 因为reset中添加了情况队列的命令
            """
            if self.this_command.parallel_id!=-1 and self.active_commands:
                # 该单行任务需要等待某个并行任务执行结束
                for command in self.active_commands:
                    if command.id==self.this_command.parallel_id and command.completed:
                        self.this_command = self.command_queue.pop(0)
                        self.this_command.start_time = time.time()
                        break
                    else:
                        continue

                pass

            else:
                self.this_command = self.command_queue.pop(0)
                self.this_command.start_time = time.time()

        if not self.this_command.completed:
            self.this_command.execute(env)

        if self.this_command.completed and self.this_command.type != 'None':
            if self.this_command.Command_ifshow:
                print(f"Command {self.this_command.type} completed, id:{self.this_command.id}.")
            self.this_command = Command('None',)
            self.this_command.completed = True
        # self.command_queue = [cmd for cmd in self.command_queue if not cmd.completed]


        # Execute all parallel commands
        current_time = time.time()
        current_parallel_id = self.this_command.id
        for command in self.active_commands:
            if not command.completed and (command.parallel_id == current_parallel_id ):
                # 判断并行任务的等待时间
                if self.this_command.type != 'None' and current_time - self.this_command.start_time >= command.wait_time :
                    command.execute(env)
            elif not command.completed and command.parallel_id==-1:
                command.execute(env)

        # Clean up completed parallel commands
        self.active_commands = [cmd for cmd in self.active_commands if not cmd.completed]





