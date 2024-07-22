import time
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ''))
import numpy as np
import pybullet as p
import functools
import pybullet_data
from utils import *
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
from User_Defined_Demos.jig.Controller import Controller,Command
from file_utils import save_PointCloud



def run_controller(grasp_system):
    grasp_system.run()
    # p.disconnect(grasp_system.connection_id)

def run_simulation(command_queue,data_queue):
    grasp_system = GraspSystem(False)
    init_sys(grasp_system)
    # load_objs(grasp_system)
    grasp_system.reset_cam_fps(30)
    grasp_system.data_queue = data_queue
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


def init_sys(sys:GraspSystem):
    sys.id_client = p.connect(p.GUI)
    p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1, shadowMapWorldSize=1, shadowMapIntensity=1,
                               physicsClientId=sys.id_client)
    p.setAdditionalSearchPath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '../Robot/urdf/'))
    # 设置数据搜索路径
    p.setAdditionalSearchPath(pybullet_data.getDataPath(), physicsClientId=sys.id_client)

    planeId = p.loadURDF("plane.urdf")

    cam_urdf = "F:\\sw\\urdf_files\\camera\\urdf\\camera.urdf"
    sys.camera = Robot(sys.id_client)
    sys.camera.load_urdf(fileName=cam_urdf, basePosition=(0, 0, 1),
                          baseOrientation=p.getQuaternionFromEuler([1.57, 0, 0]), useFixedBase=0)

    sys.env = PybulletEnv(id_client=sys.id_client)
    robot_urdf = "F:\sw\\urdf_files\i7-nofork.SLDASM\\urdf\i7-nofork.SLDASM.urdf"

    sys.env.load_robot(fileName=robot_urdf, basePosition=(-0.3, 0, 0.3), useFixedBase=True,
                        flags=p.URDF_USE_SELF_COLLISION, start=[0, 0, 0, 0, 0, 0])
    sys.gp = Gripper(sys.id_client)
    file_name = "F:\\sw\\urdf_files\\Gripper_1\\urdf\\Gripper_1.urdf"
    basePosition = [0, -0.3, 1]
    baseOrientation = p.getQuaternionFromEuler([0, 0, 0])
    sys.gp.load_urdf(fileName=file_name, basePosition=basePosition, baseOrientation=baseOrientation, useFixedBase=0)

    sys.gp.init_gripper(ifshow=True)
    constraint1 = sys.bind_cam2robot(sys.env.robots[0], sys.camera, pos_in_robot_end_link=[0, 0.0370, -0.00358],
                                      pos_in_cam_base_link=[0, 0, 0], ori_in_cam_base_link=[3.14, 0, 3.14])
    constraint2 = sys.bind_gripper2robot(sys.env.robots[0], sys.gp)
    sys.init_robot_motor()
    sys.create_virtual_cams(60)
    p.setGravity(0,0,-9.8)
    pass


def load_objs(sys:GraspSystem):
    box1_urdf = "F:\\sw\\urdf_files\\objects\\urdf\\box1.urdf"
    box2_urdf = "F:\\sw\\urdf_files\\objects\\urdf\\box2.urdf"
    cylinder1_urdf = "F:\\sw\\urdf_files\\objects\\urdf\\cylinder1.urdf"
    cylinder2_urdf = "F:\\sw\\urdf_files\\objects\\urdf\\cylinder2.urdf"
    table_urdf = "F:\\sw\\urdf_files\\objects\\urdf\\table.urdf"
    sys.table = Robot(sys.id_client)
    sys.table.load_urdf(fileName=table_urdf,
                       basePosition=[0, 0.0, 0.0], useFixedBase=True)
    # sys.box1 = Robot(sys.id_client)
    # sys.box1.load_urdf(fileName=box1_urdf,
    #                       basePosition=[0, 0.1, 0.3], useFixedBase=False)
    # sys.box2 = Robot(sys.id_client)
    # sys.box2.load_urdf(fileName=box2_urdf,
    #                    basePosition=[0.1, 0, 0.3], useFixedBase=False)
    # sys.cylinder1 = Robot(sys.id_client)
    # sys.cylinder1.load_urdf(fileName=cylinder1_urdf,
    #                    basePosition=[0, -0.1, 0.3], useFixedBase=False)
    # sys.cylinder2 = Robot(sys.id_client)
    # sys.cylinder2.load_urdf(fileName=cylinder2_urdf,
    #                    basePosition=[-0.1, 0.0, 0.3], useFixedBase=False)


    pass

if __name__ == '__main__':



    command_queue = Queue()
    data_queue = Queue()
    process = Process(target=run_simulation, args=(command_queue, data_queue))
    process.start()
    command_queue.put(Command('open_gripper', parallel=False, ))
    # command_queue.put(Command('move_arm', positions=[[0.1, -0.05, 0.6], [ 0, 0, 0, 1 ]], scale=10))
    # command_queue.put(Command('move_arm', positions=[[-0.0, 0.2, 0.5], [ -0.3824995, 0, 0, 0.9239557 ]], scale=10))
    # command_queue.put(Command('move_arm', positions=[[-0.0, -0.2, 0.4], [0.3824995, 0, 0, 0.9239557]], scale=10))

    positions,orietations = sample_sphere_points([-0.0,0,0.3], 0.3,5,randomize=False,orientation_point=None,change_orientation=True)
    print(f"positions:{positions}")
    print(f"orietations:{orietations}")


    command_queue.put(Command('wait', parallel=False, sleep_time=1))
    point_in_ee_frame = [-0.030383358162982828, 0.04983539700085212, -0.013351126934074722]
    start = [0.0018165139113621802, -0.027527539748105987, -0.014033019351242248, 0.17618879145121055, 1.684950495249885e-05, -0.07284931210576644]
    start = [-0.3825554055437463, 0.5420197603728452, -1.8933175008216347, 0.21872522339548417, -1.569584190370736, -3.140798166552206]
    for i in range(len(positions)):
        command_queue.put(Command('add_debug_sys', pos=positions[i], ori=orietations[i], lifetime=0))
        print(f'pos:{positions[i]},ori:{orietations[i]}')
        command_queue.put(Command('move_arm', positions=[positions[i], orietations[i]],point_in_ee_frame=[0,0,0], scale=10,start=start,control_type='position'))
        command_queue.put(Command('wait', parallel=False, sleep_time=1))
        command_queue.put(Command('get_end_effector_data', parallel=False, parallel_id=-1))
        data = data_queue.get()
        print(f"Received data from pybullet process: {data}")
        command_queue.put(Command('show_link_state', ))
        command_queue.put(Command('wait', parallel=False, sleep_time=1))

    # command_queue.put(Command('move_arm', positions=[[-0.0, 0.2, 0.6], [ -0.3824995, 0, 0, 0.9239557 ]], scale=10))


    command_queue.put(Command('wait', parallel=False, sleep_time=1))
    command_queue.put(Command('get_point_cloud', Command_ifshow=True,ifshow=False,))
    data = pickle.loads(data_queue.get())
    points, colors = data
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    pcd.colors = o3d.utility.Vector3dVector(colors)
    Camera.show_pcd_with_orin([pcd])
    # save_PointCloud(pcd,output_directory='./data/sphere_samples',file_prefix='objs_with_table_empty',extension='ply')
    # sys.env_hold_on()
    
    pass