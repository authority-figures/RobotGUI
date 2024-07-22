import time
import pybullet as p
from utils import *
from Environment.PybulletEnv import PybulletEnv
from RobotDir.Robot import Robot
import sys


class BaseDemo:
    def __init__(self):
        id_client = p.connect(p.GUI)
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0, physicsClientId=id_client)
        p.resetDebugVisualizerCamera(cameraDistance=2, cameraYaw=30, cameraPitch=-32,
                                     cameraTargetPosition=[0, 0, 0], physicsClientId=id_client)
        p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1, physicsClientId=id_client)
        p.setAdditionalSearchPath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '../Robot/urdf/'))
        self.env = PybulletEnv(id_client=id_client)
        self.env.load_robot(fileName="/ROBOT\\urdf\\aobo-tuopu.urdf", basePosition=(0, 0, 0),
                            flags=p.URDF_USE_SELF_COLLISION, start=[0, 0, PI / 2, 0, 0, 0], useFixedBase=1)
        p.saveBullet('base.bullet', physicsClientId=id_client)

    def env_hold_on(self):
        while True:
            self.env.update(mode=1, robot_mode=2)
            time.sleep(1 / 240.)


if __name__ == '__main__':
    demo = BaseDemo()
    if len(sys.argv) > 1:
        if sys.argv[1] == "start":
            demo.env_hold_on()



