import time
from Environment.PybulletEnvOMPL import PybulletEnvOMPL, RobotOMPL
import pybullet as p
from utils import *
from RobotDir.Robot import Robot


class OmplPlanDemo:
    # 创建pybullet连接
    def __init__(self):
        id_client_direct = p.connect(p.DIRECT)
        p.setAdditionalSearchPath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '../RobotDir/urdf/'))

        self.env_direct = PybulletEnvOMPL(id_client=id_client_direct)

        self.env_direct.load_robot(fileName="tuopu.urdf", basePosition=(0, 0, 0),
                            useFixedBase=1, flags=p.URDF_USE_SELF_COLLISION, start=[0, 0, 0, 0, 0, 0])

        self.env_direct.load_robot(fileName="aubo-i7.urdf", basePosition=(0, 0, 0),
                            flags=p.URDF_USE_SELF_COLLISION, start=[0, 0, PI / 2, 0, 0, 0])
        fixed_joint = p.createConstraint(self.env_direct.robots[0].id_robot, 5,
                                         self.env_direct.robots[1].id_robot, -1,
                                         p.JOINT_FIXED, [0, 0, 1], [0, 0, 0], [0, 0, 0],
                                         physicsClientId=id_client_direct)
        p.setCollisionFilterPair(self.env_direct.robots[0].id_robot, self.env_direct.robots[1].id_robot, 5, -1, 0,
                                 physicsClientId=id_client_direct)

        self.env_direct.add_obstacle(self.env_direct.robots[0])
        self.env_direct.set_robot(self.env_direct.robots[1])
        self.env_direct.build_state_space()

    def plan(self, start, goal):
        res, path = self.env_direct.plan_from_start_goal(start=start, goal=goal, planner_name="RRTConnect")
        return res, path

    def plot(self, path):
        id_client_gui = p.connect(p.GUI)
        self.env_gui = PybulletEnvOMPL(id_client=id_client_gui)

        self.env_gui.load_robot(fileName="tuopu.urdf", basePosition=(0, 0, 0),
                                   useFixedBase=1, flags=p.URDF_USE_SELF_COLLISION, start=[0, 0, 0, 0, 0, 0])

        self.env_gui.load_robot(fileName="aubo-i7.urdf", basePosition=(0, 0, 0),
                                   flags=p.URDF_USE_SELF_COLLISION, start=[0, 0, PI / 2, 0, 0, 0])
        fixed_joint = p.createConstraint(self.env_gui.robots[0].id_robot, 5,
                                         self.env_gui.robots[1].id_robot, -1,
                                         p.JOINT_FIXED, [0, 0, 1], [0, 0, 0], [0, 0, 0],
                                         physicsClientId=id_client_gui)
        p.setCollisionFilterPair(self.env_gui.robots[0].id_robot, self.env_gui.robots[1].id_robot, 5, -1, 0,
                                 physicsClientId=id_client_gui)
        self.env_gui.set_robot(self.env_gui.robots[1])
        self.env_gui.key_press_user_event.get_user_input([[self.env_gui.robot.id_robot], [path]])

    def env_hold_on(self):
        while True:
            self.env_gui.update()
            time.sleep(1 / 240.)


if __name__ == '__main__':
    demo = OmplPlanDemo()
    res, path = demo.plan([0, 0, PI/2, 0, 0, 0], [0.632, -0.111, 2.479, -0.796, -1.57, 2.203])
    print('res:', res)
    print('path:', path)
    if not path:
        raise TimeoutError("No solution is found! Check planner parameter!")
    demo.plot(path)
    demo.env_hold_on()



