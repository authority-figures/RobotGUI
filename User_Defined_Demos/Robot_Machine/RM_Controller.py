import pybullet as p
from RobotDir.Controller import Controller,Command
import time
import numpy as np

class RM_Command(Command):

    def __int__(self,type, parallel=False,id=None,parallel_id=0,wait_time=0,Command_ifshow=False,**kwargs):
        super(Command,self).__init__(type, parallel=False,id=None,parallel_id=0,wait_time=0,Command_ifshow=False,**kwargs)

    def execute(self, sys):
        """ Execute the command within the given environment as a coroutine. """
        if not self.generator:
            self.start_time = time.time()  # 记录开始时间
            # Decide which generator to use based on command type
            if self.type == "run_nc_codes":
                self.generator = self.run_nc_codes(sys, **self.data)
            elif self.type == "run_robot":
                self.generator = self.run_robot(sys, **self.data)
            elif self.type == "reset_robot_pos":
                self.generator = self.reset_robot_pos(sys, **self.data)
            elif self.type == "set_step_time":
                self.generator = self.set_step_time(sys, **self.data)
            else:
                # If the command type is not new, use the parent's execute logic
                super().execute(sys)
                return

        try:
            next(self.generator)
        except StopIteration:
            self.completed = True
            self.end_time = time.time()  # 记录结束时间
            self.print_execution_time()  # 打印执行时间
            
    def run_nc_codes(self, sys, **kwargs):
        """ A generator function that defines the behavior of a new task. """
        """
                重置机床各轴的位置。

                参数:
                - body_id: 机床的body ID。
                - joint_indices: 一个包含机床各轴对应的关节索引的列表。
                """

        joint_values_list = kwargs.get("joint_values_list",  None)
        if joint_values_list==None:
            joint_values_list = sys.joint_values
        mode = kwargs.get("mode", "Reset")
        tolerance = kwargs.get("tolerance", 1e-2)
        time_scale = kwargs.get("time_scale", 1.0)
        xyzVelocity = kwargs.get("xyzVelocity", 0.1)
        acVelocity = kwargs.get("acVelocity", 0.5)
        show_state = kwargs.get("show_state", True)



        # 遍历所有关节索引，并将其位置重置为0
        axes = ['A', 'C', 'X', 'Y', 'Z', 'N']  # 定义轴的顺序
        joint_indices = list(range(len(axes)))

        def has_reached_target(target_values, tolerance):

            current_positions = [p.getJointState(sys.machine.id_robot, i)[0] for i in joint_indices]
            if_return = all(not target or (abs(current - target) < tolerance) for current, target in
                            zip(current_positions, target_values))
            return if_return

        id = 0
        if mode == "Reset":
            for joint_values in joint_values_list:
                id += 1
                for i, axis_value in enumerate(joint_values):
                    if axis_value is not None and axes[i] in ['A', 'C', 'X', 'Y', 'Z']:  # 确保该轴有值
                        p.resetJointState(sys.machine.id_robot, jointIndex=i, targetValue=axis_value)
                p.stepSimulation()
                time.sleep(1 * time_scale / 240.)
                yield
                if show_state:
                    print(
                        f"{int(joint_values[axes.index('N')]) if axes.index('N') < len(joint_values) else id} NC code: {[f'{axes[idx]}:{pos:.4f}' if pos is not None else f'{axes[idx]}:None' for idx, pos in enumerate(joint_values)]}")

        else:
            for joint_values in joint_values_list:
                id += 1
                current_joints = [p.getJointState(sys.machine.id_robot, i)[0] for i in joint_indices]
                # sys.interpolation_path([current_joints,joint_values],)
                for i, axis_value in enumerate(joint_values):
                    maxVelocity = acVelocity if i in [0, 1] else xyzVelocity
                    if axis_value is not None and axes[i] in ['A', 'C', 'X', 'Y', 'Z']:  # 确保该轴有值
                        p.setJointMotorControl2(bodyUniqueId=sys.machine.id_robot,
                                                jointIndex=i,
                                                controlMode=p.POSITION_CONTROL,
                                                targetPosition=axis_value,
                                                targetVelocity=0.0,
                                                positionGain=0.2,  # KP
                                                velocityGain=0.5,  # KD
                                                force=100000,
                                                maxVelocity=maxVelocity,
                                                )
                joint_values_for_judge = np.delete(joint_values, axes.index('N'))
                while not has_reached_target(joint_values_for_judge, tolerance):
                    yield
                print(
                    f"{int(joint_values[axes.index('N')]) if axes.index('N') < len(joint_values) else id} NC code: {[f'{axes[idx]}:{pos:.4f}' if pos is not None else f'{axes[idx]}:None' for idx, pos in enumerate(joint_values)]}")


    def run_robot(self, sys, **kwargs):
        robot_goto_positions = kwargs.get("robot_goto_positions", sys.robot_goto_positions)
        start = kwargs.get("start", None)
        maxVelocity = kwargs.get("maxVelocity", 1)
        joints = kwargs.get("joints", None)
        save = kwargs.get("save", False)
        inter_scale = kwargs.get("inter_scale", 1)
        if joints is None:
            joints = sys.convert_positions_to_joints(robot_goto_positions=robot_goto_positions ,start=start,save=save)
        if inter_scale<=0:
            inter_joints = joints
        else:
            inter_joints = sys.env.robots[0].interpolation_path(joints, scale=inter_scale, add_more_end=0)
        inter_joints = inter_joints
        for id, joints in enumerate(inter_joints):

            for i, joint_position in enumerate(joints):
                # p.resetJointState(sys.env.robots[0].id_robot, i, joint_position)

                p.setJointMotorControl2(sys.env.robots[0].id_robot, i, p.POSITION_CONTROL,
                                        targetPosition=joint_position,
                                        targetVelocity=0.0,
                                        positionGain=0.5,  # KP
                                        velocityGain=0.8,  # KD
                                        force=10000,
                                        maxVelocity=maxVelocity,
                                        )
            one_point_time =  time.time()
            while not sys.env.robots[0].has_reached_target(joints, 1e-3):
                if time.time() - one_point_time > 1:
                    break
                yield

            pos, ori = sys.env.robots[0].get_end_effector_info()
            pos, ori = sys.get_point_in_workpiece2world(pos, ori,inverse=True)
            pos, ori = sys.get_point_in_workpiece2robot(pos, ori)
            pos, ori = sys.env.robots[0].calculate_ee_origin_from_target(None,None,sys.point_in_ee_frame,sys.robot_target_ori,
                                                                         inverse=True,ee_origin_world=pos,ee_orientation_world=ori)
            pos, ori = sys.get_point_in_workpiece2robot(pos, ori,inverse=True)
            print(f'run_robot:point {id + 1}/{len(inter_joints)} pos={pos},ori={ori}')

            # time.sleep(0.1)
        print(f'run_robot end with {len(inter_joints)} points')
        pass


    def reset_robot_pos(self, sys, **kwargs):
        robot_target_pos = kwargs.get("robot_target_pos", [0,0,0])
        robot_target_ori = kwargs.get("robot_target_ori", [0,0,0,1])
        inter_scale = kwargs.get("inter_scale", 1)
        start = kwargs.get("start", None)
        robot_target_pos, robot_target_ori = sys.get_point_in_workpiece2robot(robot_target_pos, robot_target_ori)
        robot_target_pos, robot_target_ori = sys.env.robots[0].calculate_ee_origin_from_target(robot_target_pos,
                                                                                                robot_target_ori,
                                                                                                sys.point_in_ee_frame,
                                                                                                sys.robot_target_ori)
        joints = sys.env.robots[0].calc_path_joints(None, None, robot_target_pos, robot_target_ori, start=start)
        inter_joints = sys.env.robots[0].interpolation_path(joints, scale=inter_scale, add_more_end=0)
        for joints in inter_joints:
            for i, joint_position in enumerate(joints):
                # p.resetJointState(self.env.robots[0].id_robot, i, joint_position)

                p.setJointMotorControl2(sys.env.robots[0].id_robot, i, p.POSITION_CONTROL,
                                        targetPosition=joint_position,
                                        targetVelocity=0.0,
                                        positionGain=0.5,  # KP
                                        velocityGain=0.8,  # KD
                                        force=1000,
                                        maxVelocity=2,
                                        )
        while not sys.env.robots[0].has_reached_target(joints, 1e-3):
            yield

        print(f'reset_robot_pos:joints={sys.env.robots[0].get_joints_states()}')

        pass

    def set_step_time(self, sys, **kwargs):
        step_time = kwargs.get("step_time", 1/240.)
        sys.step_time = step_time
        yield
        pass


class RM_Controller(Controller):

    def __int__(self):
        super(Controller,self).__init__()


