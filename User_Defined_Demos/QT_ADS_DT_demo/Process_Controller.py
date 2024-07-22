import multiprocessing
from multiprocessing import Process, Queue, Manager, Value
import pybullet as p
from Environment.PybulletEnv import PybulletEnv  # 确保从正确的位置导入你的环境类
import time
import ctypes
from queue import Empty
import pygetwindow as gw
import win32gui
# %%

# class Run:
#     def __init__(self):
#         self.value = False
class RobotMessages:
    def __init__(self, joint_states=None, end_effector_pose_and_ori=None, additional_info=None, isNone=False):
        self.joint_states = joint_states
        self.end_effector_pos_and_ori = end_effector_pose_and_ori
        self.additional_info = additional_info
        self.isNone = isNone


processes_info = {}
class Process_Controller:
    def __init__(self):
        # manager = Manager()

        self.envs : dict[str:dict]= {}
        # self.envs = manager.dict()
        self.queues = {}
        # self.processes = []
        # self.processes_info = {}




    def create_env(self, env_name, gui_enabled=True,DebugVisualizer=True):
        """创建并启动一个新的PyBullet环境"""
        command_queue = Queue()
        data_queue = Queue()  # 新增数据队列
        env_process = Process(target=self.run_pybullet_env, args=(env_name, command_queue,data_queue, gui_enabled,DebugVisualizer))
        # run = Run()
        # self.queues[env_name] = queue  # 存储队列
        self.envs[env_name] = {
            # 'process': env_process,   # 不能对process进行存储，因为process不可序列化
            'command_queue': command_queue,
            'data_queue': data_queue,
            'env': None,  # 这里稍后设置环境
            'Run': Value(ctypes.c_bool, False),  # 使用 manager.Value 来共享布尔值
            # 'Run': run,
            'win_hwnd': None,
            'win_name': None,
            'time_out': 0.1,
        }
        # self.processes.append(env_process)
        processes_info[env_name] = {
            'process': env_process,
            'alive': True
        }
        env_process.start()

    def run_pybullet_env(self,env_name, command_queue,data_queue, gui_enabled,DebugVisualizer):
        """PyBullet环境的独立进程"""
        id_client = p.connect(p.GUI if gui_enabled else p.DIRECT)
        if DebugVisualizer:
            p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0, physicsClientId=id_client)
        time.sleep(0.5)
        # 查找PyBullet窗口
        bullet_window = gw.getWindowsWithTitle('Bullet Physics') # 假设窗口标题包含“PyBullet”
        if bullet_window:
            bullet_hwnd = win32gui.FindWindow(None, bullet_window[0].title)
            self.envs[env_name]['win_hwnd'] = bullet_hwnd
            self.envs[env_name]['win_name'] = bullet_window[0].title
            print("{}:{}".format(env_name,bullet_hwnd))
        else:
            print("Failed to find the PyBullet window for environment:", env_name)


        env = PybulletEnv(id_client)
        self.envs[env_name]['env'] = env  # 储存环境引用
        # self.e[env_name] = env
        print('Env:{},已创建'.format(env_name))
        time_out = self.envs[env_name]['time_out']
        self.Run_joint_list = []
        while True:
            try:
                # cmd = command_queue.get_nowait()  # 使用非阻塞获取,当队列为空时会发出Empty异常，不会阻碍程序 | 存在问题，当创建GUI与DT连接时会卡顿
                cmd = command_queue.get(timeout=time_out)  # 使用这种方式不会造成卡顿 | 可以动态调整超时时间
            except Empty:
                cmd = None


            if cmd == 'terminate':
                p.disconnect(physicsClientId=id_client)
                print(f'env:{env_name} has been disconnected')

            elif cmd == 'Pause':
                self.envs[env_name]['Run'].value = False
                time_out = 0.1
                print("Env:{} 已暂停".format(env_name))
            elif cmd == 'reset':
                try:
                    self.Run_joint_list = []
                    if 'Robot' in env.robots_dict:
                        robot = env.robots_dict['Robot']
                        robot.set_joints_states((0,0,0,0,0,0))
                    else:
                        raise ValueError('Robot is not exist')
                except Exception as e:
                    print(f"ERROR in reset env={env_name}:{str(e)}")
                pass
            elif cmd == 'load_robot':
                file_name, base_position, use_fixed_base, flags = command_queue.get()
                env.load_robot(file_name, base_position, use_fixed_base, flags)
                env.robots_dict['Robot'] = env.robots[-1]
                print('Env:{},load robot'.format(env_name))
            elif cmd == 'Run':
                self.envs[env_name]['Run'].value = True
                time_out = 0.001
                print("Env:{} 已运行".format(env_name))
                pass
            elif cmd == 'set_pos':
                try:
                    pos = command_queue.get()[0]    # 这里会将command_queue.get()得到的数据放在一个元组中
                    if pos is not None:
                        if 'Robot' in env.robots_dict:

                            robot = env.robots_dict['Robot']
                            ori = robot.info_end_effector[1]
                            target_values = robot.get_state_from_ik(pos=pos,ori=ori)
                            robot.set_joints_states(target_values)
                        else:
                            raise ValueError('Robot is not exist')
                except Exception as e:
                    print(f"ERROR in set_pos env={env_name}:{str(e)}")

            elif cmd == 'set_ori':
                try:
                    ori = command_queue.get()[0]
                    if ori is not None:
                        if 'Robot' in env.robots_dict :
                            robot = env.robots_dict['Robot']
                            pos = robot.info_end_effector[0]
                            target_values = robot.get_state_from_ik(pos=pos,ori=ori)
                            robot.set_joints_states(target_values)
                        else:
                            raise ValueError('Robot is not exist')
                except Exception as e:
                    print(f"ERROR in set_ori env={env_name}:{str(e)}")
            elif cmd == 'set_joint_angle':
                try:
                    joint_angles = command_queue.get()[0]
                    if joint_angles is not None:
                        if 'Robot' in env.robots_dict :
                            robot = env.robots_dict['Robot']
                            robot.set_joints_states(joint_angles)
                        else:
                            raise ValueError('Robot is not exist')
                except Exception as e:
                    print(f"ERROR in set_joint_angle env={env_name}:{str(e)}")
            elif cmd == 'run_joint_angle_list_path':
                try:
                    self.Run_joint_list = command_queue.get()[0]

                except Exception as e:
                    print(f"ERROR in run_joint_angle_list_path env={env_name}:{str(e)}")

            # 运行路径列表
            if self.Run_joint_list and 'Robot' in env.robots_dict:
                try:
                    robot = env.robots_dict['Robot']
                    joint_angles = self.Run_joint_list.pop(0)
                    robot.set_joints_states(joint_angles)
                    time.sleep(1./240)
                except Exception as e:
                    print(f"ERROR in Run_joint_list env={env_name}:{str(e)}")

            if self.envs[env_name]['Run'].value:
                try:
                    if 'Robot' in env.robots_dict:
                        env.update(mode=1, robot_mode=2)
                        robot = env.robots_dict['Robot']
                        robot_message = RobotMessages(joint_states=robot.states_joints,
                                                end_effector_pose_and_ori=robot.info_end_effector,
                                                additional_info="some_info")
                    else:
                        robot_message = None
                    data_queue.put(robot_message)
                except Exception as e:
                    print(f"ERROR in sending message from env={env_name},and env is running:{str(e)}")
            # 可以处理更多命令
            else:
                try:
                    if 'Robot' in env.robots_dict:
                        env.update(mode=-1, robot_mode=0)
                        robot = env.robots_dict['Robot']
                        robot_message = RobotMessages(joint_states=robot.states_joints, end_effector_pose_and_ori=robot.info_end_effector,
                                               additional_info="Pause")
                    else:
                        robot_message = None
                    data_queue.put(robot_message)

                except Exception as e:
                    print(f"ERROR in sending message from env={env_name},and env is not running:{str(e)}")

    def send_command(self, env_name, command, *args):
        """向指定环境发送命令及参数"""
        if env_name in self.envs:
            self.envs[env_name]['command_queue'].put(command)
            self.envs[env_name]['command_queue'].put(args)

    def stop_env(self, env_name):
        """停止指定的环境"""
        if env_name in self.envs:
            # self.envs[env_name]['command_queue'].put('terminate')
            processes_info[env_name]['process'].terminate()  # 发送终止信号
            processes_info[env_name]['process'].join()  # 等待进程结束
            del self.envs[env_name]
            del processes_info[env_name]
            print(f'env:{env_name} has been terminated')

    def stop_all_envs(self):
        # 停止所有子进程
        for env_name in processes_info:
            processes_info[env_name]['process'].terminate()  # 发送终止信号
            processes_info[env_name]['process'].join()  # 等待进程结束
            print(f'env:{env_name} has been terminated')




# %%
if __name__ == '__main__':
    controller = Process_Controller()
    controller.create_env('Simulation1', gui_enabled=True)  # 创建一个带GUI的环境
    time.sleep(2)  # 给子进程时间来启动和初始化
    controller.create_env('Simulation2', gui_enabled=True)
    time.sleep(1)  # 给子进程时间来启动和初始化
    controller.send_command('Simulation1', 'load_robot', "F:\sw\\urdf_files\i7-nofork.SLDASM\\urdf\i7-nofork.SLDASM.urdf", (0, 0, 0), True, 0)
    controller.send_command('Simulation1', 'Run')
    time.sleep(2)
    controller.send_command('Simulation2', 'load_robot',
                            "F:\sw\\urdf_files\i7-nofork.SLDASM\\urdf\i7-nofork.SLDASM.urdf", (0, 0, 0), True, 0)
    controller.send_command('Simulation1', 'Pause')
    time.sleep(2)
    controller.send_command('Simulation1', 'Run')
    controller.stop_all_envs()
    while True:

        pass

    pass