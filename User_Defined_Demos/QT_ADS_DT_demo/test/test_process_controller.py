import multiprocessing
import time
from multiprocessing import Manager

processes = []
class Process_Controller:
    def __init__(self):
        manager = Manager()
        self.processes = {}



    def create_env(self, env_name):
        command_queue = multiprocessing.Queue()
        process = multiprocessing.Process(target=self.run_env, args=(command_queue,))
        process.start()
        self.processes[env_name] = {
            # 'process': process,
            'queue': command_queue
        }
        processes.append(process)
        return process

    def run_env(self, command_queue):
        print('env create')
        while True:
            command = command_queue.get()
            if command == 'Shut Down':
                break
        print("环境已关闭。")

    def send_command(self, env_name, command):
        if env_name in self.processes:
            self.processes[env_name]['queue'].put(command)

    def stop_env(self, env_name):
        self.send_command(env_name, 'Shut Down')
        if env_name in self.processes:
            self.processes[env_name]['process'].join()
            del self.processes[env_name]

    def stop_all_envs(self):
        for env_name in list(self.processes.keys()):
            self.stop_env(env_name)

# 使用示例
if __name__ == '__main__':
    controller = Process_Controller()
    process1 = controller.create_env('Simulation1')
    process2 = controller.create_env('Simulation2')
    time.sleep(10)
    controller.stop_all_envs()
