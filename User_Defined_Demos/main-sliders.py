import time
import pybullet as p
from utils import *
from Environment.PybulletEnvSlider import PybulletEnvSlider
from RobotDir.Robot import Robot


class SlidersDemo:
    # 创建pybullet连接
    def __init__(self):
        id_client = p.connect(p.GUI)
        p.setAdditionalSearchPath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '../RobotDir/urdf/'))
        self.env = PybulletEnvSlider(id_client=id_client)

        self.env.load_robot(fileName="tuopu-with-cutter-workpiece.urdf", basePosition=(0, 0, 0),
                            useFixedBase=1, flags=p.URDF_USE_SELF_COLLISION, start=[0, 0, 0, 0, 0, 0, 0], f_pinrt=True)
        self.env.robots[0].set_id_end_effector(4)

        self.env.load_robot(fileName="aubo-i7-with-holder.urdf", basePosition=(-0.2, 0, 0),
                            flags=p.URDF_USE_SELF_COLLISION, start=[0, 0, PI / 2, 0, 0, 0, 0])
        fixed_joint = p.createConstraint(self.env.robots[0].id_robot, 6, self.env.robots[1].id_robot, -1,
                                         p.JOINT_FIXED, [0, 0, 1], [0, 0, 0], [0.2, 0, 0], physicsClientId=id_client)
        p.setCollisionFilterPair(self.env.robots[0].id_robot, self.env.robots[1].id_robot, 6, -1, 0,
                                 physicsClientId=id_client)

    def env_hold_on(self):
        while True:
            self.env.update(mode=1)
            time.sleep(1 / 240.)


if __name__ == '__main__':
    demo = SlidersDemo()
    demo.env_hold_on()



