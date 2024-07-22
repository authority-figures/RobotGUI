import time
from RM_sys import RM_sys
from multiprocessing import Process,Queue
import threading
import open3d as o3d
import pickle
from User_Defined_Demos.jig.combined_sys import GraspSystem
from RobotDir.Robot import Robot
from RobotDir.utils.my_utils import *
from RobotDir.Camera import Camera
from Environment.PybulletEnv import PybulletEnv
from RobotDir.Gripper import Gripper
from RM_Controller import RM_Controller,RM_Command


def run_controller(rm_system):
    rm_system.run()
    # p.disconnect(grasp_system.connection_id)

def run_simulation(command_queue,data_queue):
    rm_system = RM_sys(True)
    init_sys(rm_system)
    # load_objs(grasp_system)
    rm_system.data_queue = data_queue
    # 开始仿真在一个新线程
    thread = threading.Thread(target=run_controller, args=(rm_system,))
    thread.start()
    while True:
        if not command_queue.empty():
            command = command_queue.get()

            rm_system.controller.add_command(command)
            time.sleep(0.1)
            if command.type == "terminate":
                break
                pass

    thread.join()

def init_sys(sys:RM_sys):
    sys.set_R_W_M_collision()
    pass



if __name__ == '__main__':

    command_queue = Queue()
    data_queue = Queue()
    process = Process(target=run_simulation, args=(command_queue, data_queue))
    process.start()


    start = [-5.632696646620288, 2.9601766673109164, 1.3607487723362872, 4.320925361170024, -0.9211075418969108, 4.710803686683021]
    # start = None
    command_queue.put(RM_Command('reset_robot_pos',id=1,inter_scale=10, robot_target_pos=[0, -0.05, 0.08],robot_target_ori=[ 0.0005629, -0.706825, 0.707388, 0.0005633 ],start=start,Command_ifshow=True))
    command_queue.put(RM_Command('wait',id=2, parallel=False,parallel_id=-1, sleep_time=1, Command_ifshow=True))
    # command_queue.put(RM_Command('reset_robot_pos', robot_target_pos=[0, -0.05, 0.08],robot_target_ori=[0.6996540305742056,  0.1023925656542926,  0.1023925656542926,0.6996540305742056],start=None))
    joints = np.load('./files/Aubo_i3_standard.npy')
    command_queue.put(RM_Command('set_step_time', id=1001, step_time=0., Command_ifshow=True, parallel=False))
    command_queue.put(RM_Command('run_nc_codes',parallel=False,id=3, joint_values_list=None, mode="Reset0", time_scale=1, tolerance=1e-3,
                                              xyzVelocity=0.5, acVelocity=2))
    command_queue.put(RM_Command('run_robot',id=4,joints=joints,inter_scale=0,save=False,parallel=True,parallel_id=3,wait_time=1,Command_ifshow=True))

    command_queue.put(RM_Command('set_step_time', id=1002, step_time=2/240., Command_ifshow=True,parallel=True,parallel_id=3,wait_time=10))
    pass