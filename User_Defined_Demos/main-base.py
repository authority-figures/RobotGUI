import time
import pybullet as p
from utils import *
from Environment.PybulletEnv import PybulletEnv
from RobotDir.Robot import Robot
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ''))

class BaseDemo:
    # 创建pybullet连接
    def __init__(self):
        id_client = p.connect(p.GUI)
        p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1, shadowMapWorldSize=1, shadowMapIntensity=1, physicsClientId=id_client)
        p.setAdditionalSearchPath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '../RobotDir/urdf/'))
        self.env = PybulletEnv(id_client=id_client)
        workpiece_urdf = "tuopu-with-cutter-workpiece.urdf"
        # robot_urdf = "aubo-i7-with-holder.urdf"
        robot_urdf = "F:\sw\\urdf_files\i7-nofork.SLDASM\\urdf\i7-nofork.SLDASM.urdf"
        self.env.load_robot(fileName=workpiece_urdf, basePosition=(0, 0, 0),
                            useFixedBase=1, flags=p.URDF_USE_SELF_COLLISION, start=[0, 0, 0, 0, 0, 0], f_pinrt=True)
        self.env.robots[0].set_id_end_effector(4)

        self.env.load_robot(fileName=robot_urdf, basePosition=(0, 0, 0),
                            flags=p.URDF_USE_SELF_COLLISION, start=[0, 0, PI/2, 0, 0, 0])
        # fixed_joint = p.createConstraint(self.env.robots[0].id_robot, 6, self.env.robots[1].id_robot, -1,
        #                                  p.JOINT_FIXED, [0, 0, 1], [0, 0, 0], [0, 0, 0], physicsClientId=id_client)

        fixed_joint = self.env.createConstraint(self.env.robots[0].id_robot, 6, self.env.robots[1].id_robot, -1,
                                         p.JOINT_FIXED, [0, 0, 1], [0, 0, 0], [0, 0, 0], physicsClientId=id_client)

        p.setCollisionFilterPair(self.env.robots[0].id_robot, self.env.robots[1].id_robot, 6, -1, 0,
                                 physicsClientId=id_client)

    def env_hold_on(self):
        while True:
            self.env.update(mode=1, robot_mode=2)
            time.sleep(1 / 240.)
            # data = self.env.robots[1].info_end_effector
            # print(data)


if __name__ == '__main__':
    demo = BaseDemo()
    demo.env_hold_on()



