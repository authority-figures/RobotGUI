import time
from Environment.PybulletEnvOMPL import PybulletEnvOMPL, RobotOMPL
import pybullet as p
from utils import *
from RobotDir.Robot import Robot


class OmplPlanDemo:
    # 创建pybullet连接
    def __init__(self):
        id_client = p.connect(p.DIRECT)
        p.setAdditionalSearchPath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '../RobotDir/urdf/'))

        self.env = PybulletEnvOMPL(id_client=id_client)

        self.env.load_robot(fileName="tuopu-with-cutter-workpiece.urdf", basePosition=(0, 0, 0),
                            useFixedBase=1, flags=p.URDF_USE_SELF_COLLISION, start=[0, 0, 0, 0, 0, 0], f_pinrt=True)
        self.env.robots[0].set_id_end_effector(4)

        self.env.load_robot(fileName="aubo-i7-with-holder.urdf", basePosition=(-0.2, 0, 0),
                            flags=p.URDF_USE_SELF_COLLISION, start=[0, 0, PI / 2, 0, 0, 0])
        fixed_joint = p.createConstraint(self.env.robots[0].id_robot, 6, self.env.robots[1].id_robot, -1,
                                         p.JOINT_FIXED, [0, 0, 1], [0, 0, 0], [0.2, 0, 0], physicsClientId=id_client)
        p.setCollisionFilterPair(self.env.robots[0].id_robot, self.env.robots[1].id_robot, 6, -1, 0,
                                 physicsClientId=id_client)

        p.setCollisionFilterPair(self.env.robots[0].id_robot, self.env.robots[1].id_robot, 6, 6, 0,
                                 physicsClientId=id_client)

        self.env.add_obstacle(self.env.robots[0])
        self.env.set_robot(self.env.robots[1])
        self.env.build_state_space()

    @timer
    def plan(self, start, goal):
        res, path = self.env.plan_from_start_goal(start=start, goal=goal, planner_name="BITstar")
        return res, path

    def plan_from_pos_ori(self, pos, ori):
        start = self.env.robots[1].get_joints_states()
        goal = list(self.env.robots[1].get_state_from_ik(pos, ori))
        goal.append(0)
        return self.plan(start, goal)

    def plot(self, path):
        id_client = p.connect(p.GUI)
        p.setAdditionalSearchPath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '../RobotDir/urdf/'))

        self.env = PybulletEnvOMPL(id_client=id_client)

        self.env.load_robot(fileName="tuopu-with-cutter-workpiece.urdf", basePosition=(0, 0, 0),
                            useFixedBase=1, flags=p.URDF_USE_SELF_COLLISION, start=[0, 0, 0, 0, 0, 0], f_pinrt=True)
        self.env.robots[0].set_id_end_effector(4)

        self.env.load_robot(fileName="aubo-i7-with-holder.urdf", basePosition=(0, 0, 0),
                            flags=p.URDF_USE_SELF_COLLISION, start=[0, 0, PI / 2, 0, 0, 0])
        fixed_joint = p.createConstraint(self.env.robots[0].id_robot, 6, self.env.robots[1].id_robot, -1,
                                         p.JOINT_FIXED, [0, 0, 1], [0, 0, 0], [0, 0, 0], physicsClientId=id_client)
        p.setCollisionFilterPair(self.env.robots[0].id_robot, self.env.robots[1].id_robot, 6, -1, 0,
                                 physicsClientId=id_client)

        self.env.set_robot(self.env.robots[1])
        self.env.key_press_user_event.get_user_input([[self.env.robot.id_robot], [path]])

    def env_hold_on(self):
        while True:
            self.env.update()
            time.sleep(1 / 240.)


if __name__ == '__main__':
    demo = OmplPlanDemo()
    res, path = demo.plan_from_pos_ori((0.120, -0.040, 0.390), [0, -0.707, 0.707, 0])
    print('res:', res)
    print('path:', path)
    if not path:
        raise TimeoutError("No solution is found! Check planner parameter!")
    demo.plot(path)
    demo.env_hold_on()



