import time
import pybullet as p
from utils import *
from Environment.PybulletQtEnv import PybulletQtEnv
from RobotDir.Robot import Robot
from Environment.PybulletQtEnv import RobotMessages
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ''))


class Digital_Twin():

    def __init__(self):
        self.env_list : dict[str:PybulletQtEnv] = {}
        self.robot_list = {}
        self.send_messages = False
        # self.creat_client("Direct",type=p.DIRECT)
        # self.creat_client("GUI")

    def creat_client(self,name,type=p.GUI):
        id_client = p.connect(type)
        p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1, shadowMapWorldSize=1, shadowMapIntensity=1,
                                   physicsClientId=id_client)
        p.setAdditionalSearchPath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '../../RobotDir/urdf/'))
        self.env = PybulletQtEnv(id_client=id_client)
        self.env_list[name] = self.env

    def load_robot(self,name="undefined",env_name="GUI"):
        robot_urdf = "F:\sw\\urdf_files\i7-nofork.SLDASM\\urdf\i7-nofork.SLDASM.urdf"
        self.env_list[env_name].load_robot(fileName=robot_urdf, basePosition=(0, 0, 0),
                            flags=p.URDF_USE_SELF_COLLISION, start=[0, 0, PI / 2, 0, 0, 0],useFixedBase=True)
        self.robot_list[name] = self.env_list[env_name].robots[-1]


    def go_to_point(self,*args,**kwargs):
        target_values = self.robot_list["undefined"].get_state_from_ik(*args,**kwargs)
        self.robot_list["undefined"].set_joints_states(target_values)

    def get_joint_states(self):
        return self.robot_list["undefined"].states_joints
        pass

    def get_messages(self):
        robot_messages = RobotMessages(joint_states=self.robot_list["undefined"].states_joints, end_effector_pose_and_ori=self.robot_list["undefined"].info_end_effector,
                                 additional_info="some_info")
        return robot_messages
        pass

    def run(self):
        self.env_hold_on()
        pass

    def shut_down(self):
        p.disconnect(physicsClientId=self.env.id_client)

    def set_send_messages(self, status):
        self.send_messages = status

    def env_hold_on(self,Time=-1):
        start = time.time()
        while True:
            self.env.update(mode=1, robot_mode=2)
            # 假设self.robot.states_joints
            if self.send_messages:
                # self.env.signal_emitter.joint_states_updated.emit(self.get_joint_states())
                self.env.signal_emitter.robot_messages_updated.emit(self.get_messages())

            self.env.execute_commands()  # 在每次更新中执行命令

            time.sleep(1 / 240.)
            if Time<0:
                pass
            elif time.time()-start>Time:
                self.shut_down()




if __name__ == '__main__':
    D1 = Digital_Twin()
    D1.creat_client(name="GUI")
    D1.run()
    pass