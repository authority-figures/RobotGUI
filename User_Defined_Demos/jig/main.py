import time
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ''))
import numpy as np
import pybullet as p
import sys
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ''))
import functools
from utils import *
from Environment.PybulletEnv import PybulletEnv
from RobotDir.Robot import Robot
from RobotDir.Machine import Machine
from RobotDir.Gripper import Gripper
import pybullet_data
from RobotDir.Camera import Camera
from combined_sys import GraspSystem
import threading
from Controller import Controller,Command
import multiprocessing
from multiprocessing import Process,Queue
# def run_controller(system):
#     while system.running:
#         system.controller.execute_commands(system)
#         system.update_cam_pos()
#         if system.RGB_camera.img_data:
#             system.RGB_camera.display_depth_image(system.RGB_camera.img_data, 0, 1)
#         system.sys_update()


def main():
    controller = Controller()
    grasp_system = GraspSystem()
    grasp_system.reset_cam_fps(30)
    grasp_system.create_box()
    grasp_system.controller = controller

    # Add commands
    # grasp_system.controller.add_command(Command('move_by_delta', delta_pos=[0.1, 0, 0], scale=10))
    # grasp_system.controller.add_command(Command('open_gripper', parallel=False, ))
    # grasp_system.controller.add_command(Command('move_arm', parallel=False, positions=[[-0.5, -0.3, 0.139], [0, 0, 0, 1]], scale=10))
    # grasp_system.controller.add_command(Command('close_gripper', parallel=False, parallel_id=2))
    # grasp_system.controller.add_command(Command('wait', parallel=False, sleep_time=5))
    grasp_system.controller.add_command(Command('move_arm', parallel=False, positions=[[-0, -0, 0.5], [0, 0, 0, 1]], scale=10))
    grasp_system.controller.add_command(Command('wait', parallel=False, sleep_time=2))
    grasp_system.controller.add_command(
        Command('move_arm', parallel=False, positions=[[-0.3, 0.5, 0.4], [ -0.3535534, -0.3535534, 0.1464466, 0.8535534 ]], scale=10))
    grasp_system.run()
    grasp_system.controller.add_command(Command('open_gripper', parallel=False, parallel_id=5, wait_time=3))
    pass

def test_thread():
    grasp_system = GraspSystem()
    grasp_system.reset_cam_fps(30)
    grasp_system.create_box()

    # 开始仿真在一个新线程
    thread = threading.Thread(target=run_controller, args=(grasp_system,))
    thread.start()

    # 在主线程中添加命令

    grasp_system.controller.add_command(Command('open_gripper', parallel=False, ))
    grasp_system.controller.add_command(Command('move_arm', parallel=False, positions=[[-0.5, -0.3, 0.139], [0, 0, 0, 1]], scale=10))
    grasp_system.controller.add_command(Command('close_gripper', parallel=False, parallel_id=2))
    grasp_system.controller.add_command(Command('wait', parallel=False, sleep_time=5))
    grasp_system.controller.add_command(Command('move_arm', parallel=False, positions=[[-0, -0, 0.5], [0, 0, 0, 1]], scale=30))
    grasp_system.controller.add_command(Command('open_gripper', parallel=False, parallel_id=5, wait_time=3))

    # 当需要停止时
    # grasp_system.running = False
    thread.join()  # 等待线程完成

    pass
def run_controller(grasp_system):

    grasp_system.run()
    # p.disconnect(grasp_system.connection_id)

def add_commands_to_queue(command_queue):
    command_queue.put(Command('open_gripper'))
    command_queue.put(Command('move_arm', positions=[[-0.5, -0.3, 0.139], [0, 0, 0, 1]], scale=10))
    command_queue.put(Command('close_gripper', parallel_id=2))
    command_queue.put(Command('wait', sleep_time=5))
    command_queue.put(Command('move_arm', positions=[[-0, -0, 0.5], [0, 0, 0, 1]], scale=30))
    command_queue.put(Command('open_gripper', parallel_id=5, wait_time=3))


def muti_env(command_queue,data_queue):
    grasp_system = GraspSystem()
    grasp_system.reset_cam_fps(30)
    grasp_system.data_queue = data_queue
    # grasp_system.create_box()
    # 开始仿真在一个新线程
    thread = threading.Thread(target=run_controller, args=(grasp_system,))
    thread.start()
    while True:
        if not command_queue.empty():
            command = command_queue.get()

            grasp_system.controller.add_command(command)
            time.sleep(0.1)
            if command.type == "terminate":
                break
                pass

    thread.join()



    pass

def input_handler(command_queue,data_queue):
    try:
        while True:
            user_input = input("Enter command or 'exit' to quit: ")
            if user_input == "exit":
                command_queue.put(Command('terminate',parallel=True,parallel_id=-1))
                # break
            elif user_input == 'move_arm_x+':
                command_queue.put(Command('move_by_delta', delta_pos=[0.01,0,0],delta_ori=[0,0,0,0], scale=10))
            elif user_input == 'move_arm_x-':
                command_queue.put(Command('move_by_delta', delta_pos=[-0.01,0,0], scale=10))
            elif user_input == 'move_arm_y+':
                command_queue.put(Command('move_by_delta', delta_pos=[0,0.01,0], scale=10))
            elif user_input == 'move_arm_y-':
                command_queue.put(Command('move_by_delta', delta_pos=[0,-0.01,0], scale=10))
            elif user_input == 'reset':
                command_queue.put(Command('reset',parallel=True,parallel_id=-1))
            elif user_input == 'reset_random':
                random_pos = np.random.uniform(-0.1, 0.1, size=(3,))
                new_pos = random_pos + np.array([-0.5,-0.3,0.2])
                command_queue.put(Command('reset', origin_pos=new_pos, parallel=True, parallel_id=-1))
            elif user_input == 'get_data':
                command_queue.put(Command('get_end_effector_data',parallel=False,parallel_id=-1))
                # time.sleep(2)
                data = data_queue.get()
                print(f"Received data from pybullet process: {data}")




            elif user_input == 'move_arm-':
                command_queue.put(Command('move_arm', positions=[[-0, -0, 0.5], [0, 0, 0, 1]], scale=20))
    except Exception as e:
        print(f"Input thread error: {e}")

    pass
def test_multiprocessing():

    command_queue = Queue()
    data_queue = Queue()

    # pool = multiprocessing.Pool(processes=4)
    process = Process(target=muti_env , args=(command_queue,data_queue))
    process.start()

    time.sleep(2)

    input_thread = threading.Thread(target=input_handler, args=(command_queue,data_queue))
    input_thread.start()
    input_thread.join()
    print('thread end')
    process.join()
    # command_queue.put(Command('move_by_delta', delta_pos=[0.01, 0, 0], scale=10))
    # command_queue.put(Command('open_gripper'))
    # command_queue.put(Command('move_arm', positions=[[-0.5, -0.3, 0.139], [0, 0, 0, 1]], scale=10))
    # command_queue.put(Command('close_gripper', parallel_id=2))
    # command_queue.put(Command('wait', sleep_time=5))
    # command_queue.put(Command('move_arm', positions=[[-0, -0, 0.5], [0, 0, 0, 1]], scale=30))
    # command_queue.put(Command('open_gripper', parallel_id=5, wait_time=3))
    # 主线程继续添加命令

    # 添加更多命令...
    # try:
    #     while True:
    #         # 从控制台接收命令
    #         user_input = input("Enter command or 'exit' to quit: ")
    #         if user_input == "exit":
    #             command_queue.put(Command('terminate'))  # 发送终止命令
    #             break
    #         elif user_input == 'move_arm_x+':
    #             command_queue.put(Command('move_by_delta', delta_pos=[0.01, 0, 0], scale=10))
    #         elif user_input == 'move_arm_x-':
    #             command_queue.put(Command('move_by_delta', delta_pos=[-0.01, 0, 0], scale=10))
    #         elif user_input == 'move_arm_y+':
    #             command_queue.put(Command('move_by_delta', delta_pos=[0, 0.01, 0], scale=10))
    #         elif user_input == 'move_arm_y-':
    #             command_queue.put(Command('move_by_delta', delta_pos=[0, -0.01, 0], scale=10))
    #         elif user_input == 'reset':
    #             command_queue.put(Command('reset',parallel=True,parallel_id=-1))
    #
    #         elif user_input == 'move_arm-':
    #             command_queue.put(Command('move_arm', positions=[[-0, -0, 0.5], [0, 0, 0, 1]], scale=30))
    #
    # finally:
    #     process.join()  # 确保子进程完成





if __name__ == '__main__':
    # main()
    # test_thread()
    #
    test_multiprocessing()

    pass