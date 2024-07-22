from utils import *
import pybullet as p
from Environment.PybulletEnv import PybulletEnv
from Environment.PybulletEnvSlider import PybulletEnvSlider
from RobotDir.Robot import Robot


class PathDemo:
    def __init__(self):
        self.xyz_machine, self.xyz_robot, self.angle_robot = self.init_path()

    def load_robot(self, robot1: Robot, robot2: Robot):
        self.aubo = robot1
        self.tuopu = robot2

    def get_robot_path(self):
        pos = self.xyz_robot
        ori = [p.getQuaternionFromEuler(i) for i in self.angle_robot]
        return [list(self.aubo.get_state_from_ik(pos_, ori_,)) for pos_, ori_ in zip(pos, ori)]

    def get_machine_path(self):
        pos = self.xyz_machine
        return [list(self.tuopu.get_state_from_ik(pos_, None, )) for pos_ in pos]

    @staticmethod
    def init_path():
        theta = np.linspace(PI, PI/2, 100)
        a1, b1, a2, b2 = [0.180-0.005, 0.060-0.005, 0.184, 0.064]
        x0, y0, z0 = [0.3, -0.04, 0.39]
        path1 = [[np.cos(i)*a1 + x0,
                  np.sin(i)*b1 + y0,
                  z0] for i in theta]  # machine tool
        path1_ = [[np.cos(i) * a1 + x0 + np.cos(np.arctan(b1 * np.sin(i) / -a1 / np.cos(i))) * 0.01,
                  np.sin(i) * b1 + y0 + np.cos(np.arctan(b1 * np.sin(i) / -a1 / np.cos(i))) * 0.01,
                  z0] for i in theta]  # machine tool
        path2 = [[np.cos(i)*a2 + x0, np.sin(i)*b2 + y0, z0] for i in theta]  # Robot
        ori2 = [[-PI/2, 0, PI-(PI/2-np.arctan(-b2/a2/np.tan(i)))] for i in theta]  # euler angles (ZYX而不是XYZ)
        return [path1, path2, ori2]


class SynCtrlDemo:
    def __init__(self):
        id_client = p.connect(p.GUI)
        p.setAdditionalSearchPath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '../RobotDir/urdf/'))
        self.env = PybulletEnv(id_client=id_client)

        self.env.load_robot(fileName="tuopu-with-cutter-workpiece.urdf", basePosition=(0, 0, 0),
                            useFixedBase=1, flags=p.URDF_USE_SELF_COLLISION, start=[0, 0, 0, 0, 0, 0, 0], f_pinrt=True)
        self.env.robots[0].set_id_end_effector(4)

        self.env.load_robot(fileName="aubo-i7-with-holder.urdf", basePosition=(-0.2, 0, 0),
                            flags=p.URDF_USE_SELF_COLLISION, start=[0, 0, PI / 2, 0, 0, 0, 0])
        fixed_joint = p.createConstraint(self.env.robots[0].id_robot, 6, self.env.robots[1].id_robot, -1,
                                         p.JOINT_FIXED, [0, 0, 1], [0, 0, 0], [0.2, 0, 0], physicsClientId=id_client)
        p.setCollisionFilterPair(self.env.robots[0].id_robot, self.env.robots[1].id_robot, 6, -1, 0,
                                 physicsClientId=id_client)

        p.setCollisionFilterPair(self.env.robots[0].id_robot, self.env.robots[1].id_robot, 6, 6, 0,
                                 physicsClientId=id_client)

        pather = PathDemo()
        pather.load_robot(self.env.robots[1], self.env.robots[0])

        goal_xyz = (0.120, -0.040, 0.390)
        goal_ori = [0, -0.707, 0.707, 0]
        self.env.robots[1].set_joints_states(self.env.robots[1].get_state_from_ik(goal_xyz, goal_ori))

        path_aubo = pather.get_robot_path()
        path_tuopu = pather.get_machine_path()

        self.env.key_press_user_event.get_user_input([[0, 1], [path_tuopu, path_aubo]])

    def env_hold_on(self):
        while True:
            self.env.update(mode=1)
            time.sleep(1 / 240.)


if __name__ == '__main__':
    demo = SynCtrlDemo()
    demo.env_hold_on()


