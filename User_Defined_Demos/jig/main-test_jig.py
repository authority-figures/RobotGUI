import time
import pybullet as p
from utils import *
from Environment.PybulletEnv import PybulletEnv
from RobotDir.Robot import Robot
from RobotDir.Machine import Machine
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ''))

class BaseDemo:
    # 创建pybullet连接
    def __init__(self):
        id_client = p.connect(p.GUI)
        p.setGravity(0,0,-10,physicsClientId=id_client)
        p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1, shadowMapWorldSize=1, shadowMapIntensity=1, physicsClientId=id_client)
        p.setAdditionalSearchPath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '../Robot/urdf/'))
        self.env = PybulletEnv(id_client=id_client)
        box_urdf = "F:\\python\\RobotGUI_2.1\\User_Defined_Demos\\jig\\urdf_files\\box.urdf"
        machine_file_name = "F:\sw\\urdf_files\c501-simple.SLDASM\\urdf\c501-simple.SLDASM.urdf"
        robot_urdf = "F:\sw\\urdf_files\i7-nofork.SLDASM\\urdf\i7-nofork.SLDASM.urdf"
        self.env.load_robot(fileName=machine_file_name, basePosition=(0, 0, 0),
                            useFixedBase=1, flags=0, start=[0, 0, 0, 0, 0, 0], f_pinrt=True,Robot_class=Machine)

        box_id = p.loadURDF(fileName=box_urdf,basePosition=[0.2,0.0,1],useFixedBase=0)
        self.env.robots[0].set_id_end_effector(4)
        # box_id = self.env.robots[0].add_workpiece_to_machine(box_urdf,position=[0,0,0])


        # fixed_joint = p.createConstraint(self.env.robots[0].id_robot, 6, self.env.robots[1].id_robot, -1,
        #                                  p.JOINT_FIXED, [0, 0, 1], [0, 0, 0], [0, 0, 0], physicsClientId=id_client)

        # fixed_joint = self.env.createConstraint(self.env.robots[0].id_robot, 1, self.env.robots[1].id_robot, -1,
        #                                  p.JOINT_FIXED, [0, 0, 0], [-0.2, -0.2, 0], [0, 0, 0], physicsClientId=id_client)


        p.setCollisionFilterPair(self.env.robots[0].id_robot, box_id, -1, -1, 0,
                                 physicsClientId=id_client)
        p.setCollisionFilterPair(self.env.robots[0].id_robot, box_id, 0, -1, 0,
                                 physicsClientId=id_client)
        # p.setCollisionFilterPair(self.env.robots[0].id_robot, self.env.robots[1].id_robot, 1, -1, 0,
        #                          physicsClientId=id_client)
        #
        # p.setCollisionFilterPair(self.env.robots[1].id_robot, self.env.robots[0].id_robot, -1, 0, 0,
        #                          physicsClientId=id_client)

    def env_hold_on(self):
        while True:
            self.env.update(mode=1, robot_mode=2)
            time.sleep(1 / 240.)


if __name__ == '__main__':
    demo = BaseDemo()
    demo.env_hold_on()



