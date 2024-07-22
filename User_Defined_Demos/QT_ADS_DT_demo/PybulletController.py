import time
import pybullet as p
from utils import *
from Environment.PybulletQtEnv import PybulletQtEnv
from RobotDir import Robot
from Environment.PybulletQtEnv import RobotMessages
from PyQt5.QtCore import QThread
from multiprocessing import Process, Queue
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ''))




class SimulationThread(QThread):
    def __init__(self, env, duration=-1):
        super().__init__()
        self.env = env
        self.duration = duration
        self.running = True

    def run(self):
        start_time = time.time()
        while self.running and (time.time() - start_time < self.duration or self.duration == -1):
            self.env.update(mode=1, robot_mode=2)
            time.sleep(1/240)  # 控制更新频率

    def stop(self):
        self.running = False


class Pybullet_Controller():

    def __init__(self):
        self.env_dict : dict[str:PybulletQtEnv] = {}
        self.env_threads : dict[str:SimulationThread] = {}
        self.queues : dict[str:Queue] = {}

        self.robot_dict = {}
        self.send_messages = False
        # self.creat_client("Direct",type=p.DIRECT)
        # self.creat_client("GUI")

    def creat_client(self,env_name,type="GUI"):

        if type == "GUI":
            # Ensure only one GUI instance is created
            if any(env.type == 'GUI' for env in self.env_dict.values()):
                raise Exception("A GUI client has already been created")
            id_client = p.connect(p.GUI)
        elif type == "DIRECT":
            id_client = p.connect(p.DIRECT)
        else:
            raise TypeError("请选择正确的pybullet环境类型")
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0, physicsClientId=id_client)
        p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1, shadowMapWorldSize=1, shadowMapIntensity=1,
                                   physicsClientId=id_client)
        p.setAdditionalSearchPath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '../../RobotDir/urdf/'))
        self.env = PybulletQtEnv(id_client=id_client)
        self.env.type = type
        self.env_dict[env_name] = self.env
        # 为环境创建并存储线程，但不立即启动
        self.env_threads[env_name] = SimulationThread(self.env)

    def get_env(self, env_name):
        return self.env_dict.get(env_name)

    def load_robot(self,robot_name="undefined",env_name="GUI",file_path="F:\sw\\urdf_files\i7-nofork.SLDASM\\urdf\i7-nofork.SLDASM.urdf"):
        if self.env_dict:

            self.env_dict[env_name].load_robot(fileName=file_path, basePosition=(0, 0, 0),
                                               flags=p.URDF_USE_SELF_COLLISION, start=[0, 0, PI / 2, 0, 0, 0], useFixedBase=True)
            self.env_dict[env_name].robots_dict[robot_name] = self.env_dict[env_name].robots[-1]

            self.robot_dict[robot_name] = self.env_dict[env_name].robots[-1]
        else:
            raise ValueError("{}该环境还未创建".format(env_name))

    def go_to_point(self,*args,robot_name="undefined",env_name="GUI",**kwargs):
        robot = self.env_dict[env_name].robot_dict[robot_name]
        target_values = robot.get_state_from_ik(*args, **kwargs)
        robot.set_joints_states(target_values)



    def get_messages(self,robot_name="undefined",env_name="GUI"):
        robot = self.env_dict[env_name].robot_dict[robot_name]
        robot_messages = RobotMessages(joint_states=robot.states_joints, end_effector_pose_and_ori=robot.info_end_effector,
                                       additional_info="some_info")
        return robot_messages
        pass

    def run(self):
        # 启动所有环境的仿真线程
        for thread in self.env_threads.values():
            thread.start()
        pass

    def stop_all(self):
        # 停止所有仿真线程
        for thread in self.env_threads.values():
            thread.stop()
            thread.wait()  # 等待线程安全结束

    def shut_down(self):
        p.disconnect(physicsClientId=self.env.id_client)

    def set_send_messages(self, status):
        self.send_messages = status

    def env_hold_on(self,Time=-1):
        start = time.time()
        while True:
            for env in self.env_dict.values():
                env.update(mode=1, robot_mode=2)
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



class Pybullet_Calc_Manager:
    # 用于管理Direct与GUI环境，这个环境基本是一起工作的
    def __init__(self, controller):
        self.controller = controller
        self.direct_env = None
        self.gui_env = None

    def setup_Manager(self):
        # 从controller获取direct和GUI环境
        self.direct_env = self.controller.get_env("Direct")
        self.gui_env = self.controller.get_env("GUI")

    def calculate_and_visualize_path(self, start_pos, target_pos):
        # 使用direct环境计算路径
        path = self.direct_env.calculate_path(start_pos, target_pos)
        # 使用GUI环境展示路径
        self.gui_env.visualize_path(path)





if __name__ == '__main__':
    controller = Pybullet_Controller()
    controller.creat_client(env_name="GUI")
    # controller.creat_client(env_name="DT")
    controller.load_robot(robot_name="undefined",env_name="GUI")
    controller.run()

    pass