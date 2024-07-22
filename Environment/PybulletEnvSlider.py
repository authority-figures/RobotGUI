from utils import *
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ''))
from PybulletEnv import PybulletEnv
import pybullet as p
from RobotDir.Robot import Robot


class PybulletEnvSlider(PybulletEnv):
    f_sliders_modified = False
    f_robot_moved = False

    def add_slider(self, name, lower, upper, val_curr):
        idx = p.addUserDebugParameter(name, lower, upper, val_curr, physicsClientId=self.id_client)
        return idx

    def create_user_sliders(self):
        p.removeAllUserParameters(physicsClientId=self.id_client)
        self.val_sliders = []
        self.ids_sliders = []
        for robot in self.robots:
            vals = []
            ids = []
            for id_joint, val_curr in zip(robot.ids_avail_joints, robot.states_joints):
                name = robot.info_joints[id_joint][0]
                lower = robot.info_joints[id_joint][1]
                upper = robot.info_joints[id_joint][2]
                if isinstance(val_curr, list):
                    val_curr = val_curr[0]
                idx = p.addUserDebugParameter(name, lower, upper, val_curr, physicsClientId=self.id_client)
                vals.append(val_curr)
                ids.append(idx)
            self.val_sliders.append(vals)
            self.ids_sliders.append(ids)

    def get_sliders_modified(self):
        for robot, ids, vals in zip(self.robots, self.ids_sliders, self.val_sliders):
            states_joints = []
            for id_joint, i, val in zip(robot.ids_avail_joints, ids, vals):
                val_curr = p.readUserDebugParameter(i, physicsClientId=self.id_client)
                if int(val * 1000) != int(val_curr * 1000):
                    self.f_sliders_modified = True
                states_joints.append(val_curr)
            robot.set_joints_states(states_joints)

    def check_robot_move(self):
        for robot, vals in zip(self.robots, self.val_sliders):
            for val_move, val in zip(robot.get_joints_states(), vals):
                if int(val * 1000) != int(val_move * 1000):
                    return True
        return False

    def update(self, mode=0, robot_mode=1, sliders_mode=True):
        if self.ids_sliders and sliders_mode:
            self.get_sliders_modified()

        super().update(mode=mode, robot_mode=robot_mode)

        if self.f_sliders_modified and sliders_mode:
            self.create_user_sliders()
        if not self.ids_sliders and sliders_mode:
            self.create_user_sliders()
        if self.check_robot_move():
            self.create_user_sliders()
        self.f_sliders_modified = False
