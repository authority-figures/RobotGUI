import multiprocessing
import pybullet as p
from multiprocessing import Process, Queue


def pybullet_env_process(queue, gui_enabled):
    """独立进程中运行的PyBullet环境"""
    physics_client = p.connect(p.GUI if gui_enabled else p.DIRECT)
    p.setGravity(0, 0, -10)  # 示例设置重力
    while True:
        cmd = queue.get()
        if cmd == 'STOP':
            break
        # 这里可以添加更多的处理命令的逻辑
        p.stepSimulation()
    p.disconnect()


class Pybullet_Controller:
    def __init__(self):
        self.envs = {}
        self.queues = {}

    def create_env(self, env_name, gui_enabled=True):
        """创建一个新的PyBullet环境"""
        queue = Queue()
        process = Process(target=pybullet_env_process, args=(queue, gui_enabled))
        self.envs[env_name] = process
        self.queues[env_name] = queue
        process.start()

    def send_command(self, env_name, command):
        """向指定环境发送命令"""
        if env_name in self.queues:
            self.queues[env_name].put(command)

    def stop_env(self, env_name):
        """停止指定的环境"""
        self.send_command(env_name, 'STOP')
        if env_name in self.envs:
            self.envs[env_name].join()
            del self.envs[env_name]
            del self.queues[env_name]

    def stop_all_envs(self):
        """停止所有环境"""
        for env_name in list(self.envs.keys()):
            self.stop_env(env_name)

    def __del__(self):
        """确保所有环境被正确清理"""
        self.stop_all_envs()

if __name__ == '__main__':
    controller = Pybullet_Controller()
    controller.create_env('Simulation1', gui_enabled=True)  # 创建一个带GUI的环境
    controller.create_env('Simulation2', gui_enabled=True)

    # 模拟发送一些命令
    import time
    while True:
        pass

    # 停止环境
    controller.stop_all_envs()
