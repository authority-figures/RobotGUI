from PyQt5.QtCore import QObject, pyqtSignal
from Environment.PybulletEnv import PybulletEnv




class RobotMessages:
    def __init__(self, joint_states, end_effector_pose_and_ori, additional_info=None):
        self.joint_states = joint_states
        self.end_effector_pos_and_ori = end_effector_pose_and_ori
        self.additional_info = additional_info
class SignalEmitter(QObject):
    joint_states_updated = pyqtSignal(list)
    robot_messages_updated = pyqtSignal(RobotMessages)



class PybulletQtEnv(PybulletEnv):

    def __init__(self,id_client):

        PybulletEnv.__init__(self, id_client=id_client)

        self.signal_emitter = SignalEmitter()
        self.commands = []  # 命令队列
        self.robots_dict = {}
        self.type = None

    def add_command(self, command):
        self.commands.append(command)

    def execute_commands(self):
        while self.commands:
            command = self.commands.pop(0)  # 获取并移除队列中的第一个命令
            # 执行命令
            command()



    def update(self, mode=1, robot_mode=2):
        # 这里调用父类的update方法进行更新
        super().update(mode, robot_mode)

