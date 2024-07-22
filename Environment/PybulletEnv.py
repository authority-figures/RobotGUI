import pybullet as p
from typing import List

from utils import *
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ''))
import time
from RobotDir.Robot import Robot
from KeyPressUserEvent import KeyPressUserEvent
from Exception.RobotExceptions import *




class PybulletEnv:

    def __init__(self, id_client):
        self.id_client = id_client
        self.robots: List[Robot] = []
        self.robots_dict = {}

        self.num_robots = -1
        self.ids_sliders = []
        self.val_sliders = []
        self.key_press_user_event = KeyPressUserEvent(id_client)

    def load_robot(self, fileName, basePosition=(0, 0, 0), useFixedBase=0, flags=0, start=None, f_pinrt=False,Robot_class=Robot):
        robot = Robot_class(id_client=self.id_client)
        robot.f_print = f_pinrt
        robot.load_urdf(fileName=fileName, basePosition=basePosition, useFixedBase=useFixedBase, flags=flags)
        if start is None:
            start = [0]*robot.num_avail_joints
        robot.set_joints_states(start)
        self.robots.append(robot)
        self.num_robots += 1

    def add_robots_list(self, robots_list):
        self.robots = robots_list
        self.num_robots = len(robots_list)

    def collision_exclude(self, id_r1: int, id_r2: int, ids_link_r1: List[int], ids_link_r2: List[int]):
        for ids1 in ids_link_r1:
            for ids2 in ids_link_r2:
                p.setCollisionFilterPair(id_r1, id_r2, ids1, ids2, enableCollision=0,
                                         physicsClientId=self.id_client)

    def get_collision_points(self):
        self.update(0, robot_mode=-1)
        points = p.getContactPoints(physicsClientId=self.id_client)
        if len(points) > 0:
            return [True, points]
        else:
            return [False, points]

    def get_close_points(self, robot1_link1: list, robot2_link2: list, distance=INF):
        try:
            closest_points = p.getClosestPoints(robot1_link1[0], robot2_link2[0], distance,
                                                robot1_link1[1], robot2_link2[1],
                                                physicsClientId=self.id_client)
        except IndexError:
            closest_points = p.getClosestPoints(robot1_link1[0], robot2_link2[0], distance,
                                                physicsClientId=self.id_client)
        except TypeError:
            raise TypeError("输入的参数应该为, (list, list, float)")
        return closest_points

    def find_robot_by_id(self, robot_id):
        """
        在self.robots列表中查找具有特定id的Robot对象。

        参数:
        - robot_id: 要查找的Robot对象的id。

        返回:
        - 找到的Robot对象，如果没有找到则返回None。
        """
        for robot in self.robots:
            if robot.id_robot == robot_id:
                return robot
        # 如果循环完成后没有找到匹配的Robot，抛出异常
        raise RobotNotFoundException(robot_id)

    def createConstraint(self,parentBodyUniqueId=None, parentLinkIndex=-1, childBodyUniqueId=None, childLinkIndex=-1, jointType=p.JOINT_FIXED, jointAxis=[0, 0, 0], parentFramePosition=[0, 0, 0], childFramePosition=[0, 0, 0], **kwargs):
        '''
        通用型创建约束
        parentFramePosition: 固定目标点在父link坐标系下的位置
        childFramePosition: 固定目标点在子link坐标系下的位置
        :param args:
        :param kwargs:
            原参数parentBodyUniqueId,parentLinkIndex,childBodyUniqueId,childLinkIndex,jointType,jointAxis,
            parentFramePosition: 固定目标点在父链接的质心坐标系下的坐标,
            childFramePosition: 固定目标点在子链接的质心坐标系下的坐标,
            *parentFrameOrientation,*childFrameOrientation,*physicsClientId
        :return:constraintId
        '''


        # 获取父link的质心坐标
        try:
            if parentLinkIndex == -1:
                parent_robot = self.find_robot_by_id(parentBodyUniqueId)
                parent_mass_coordinate = parent_robot.baseFramePosition
            else:
                parent_mass_coordinate = Robot.get_com_in_link_frame(parentBodyUniqueId, parentLinkIndex,
                                                                     physicsClientId=self.id_client)

            if childLinkIndex == -1:
                child_robot = self.find_robot_by_id(childBodyUniqueId)
                child_mass_coordinate = child_robot.baseFramePosition
            else:
                child_mass_coordinate = Robot.get_com_in_link_frame(childBodyUniqueId, childLinkIndex,
                                                                    physicsClientId=self.id_client)
        except RobotNotFoundException as e:

            print(e)
            raise  # 可以选择重新抛出异常，如果你希望调用者知道这个错误发生了

        new_parentFramePosition = [x-y for x,y in zip(parentFramePosition,parent_mass_coordinate)]

        new_childFramePosition = [x-y for x,y in zip(childFramePosition,child_mass_coordinate)]

        parentFramePosition= new_parentFramePosition
        childFramePosition = new_childFramePosition

        constraintId = p.createConstraint(parentBodyUniqueId, parentLinkIndex, childBodyUniqueId, childLinkIndex, jointType, jointAxis, parentFramePosition, childFramePosition, **kwargs)
        return constraintId
        pass

    def update(self, mode=0, robot_mode=1):
        self.key_press_user_event.get_key_press(self)

        for robot in self.robots:
            robot.update(robot_mode)
        if mode == 0:
            p.performCollisionDetection(physicsClientId=self.id_client)
        elif mode == 1:
            p.stepSimulation(physicsClientId=self.id_client)
        elif mode == -1:
            pass
