import pybullet as p
import os
import pymeshlab

def convert_stl_to_obj(stl_file, obj_file):

    ms = pymeshlab.MeshSet()
    ms.load_new_mesh(stl_file)
    ms.save_current_mesh(obj_file)

def vhacd_decomposition(name_in, name_out, name_log, resolution=1000000, depth=1024, concavity=0.0001):
    params = {
        'resolution': resolution,
        'depth': depth,
        'concavity': concavity,
        'planeDownsampling': 4,
        'convexhullDownsampling': 4,
        'alpha': 0.005,
        'beta': 0.005,
        'gamma': 0.0001,
        'pca': 0,
        'mode': 0,  # 0=voxel based, 1=tetrahedron based
        'maxNumVerticesPerCH': 256,
        'minVolumePerCH': 0.0001,
    }
    p.vhacd(name_in, name_out, name_log, **params)

if __name__ == "__main__":
    # 定义单个STL和OBJ文件的路径
    stl_file_path = "F:/python/RobotGUI_2.1/User_Defined_Demos/jig/urdf_files/stl_models/Link_C_simple.STL"
    obj_file_path = "F:/python/RobotGUI_2.1/User_Defined_Demos/jig/urdf_files/stl_models/Link_C_simple.obj"
    convert_stl_to_obj(stl_file_path, obj_file_path)


    # 确保输出目录存在
    os.makedirs(os.path.dirname(obj_file_path), exist_ok=True)

    # 设置PyBullet连接
    p.connect(p.DIRECT)

    # 定义输出的VHACD文件路径和日志文件
    name_in = obj_file_path
    name_out = obj_file_path.replace(".obj", "_vhacd.obj")
    name_log = "vhacd_log.txt"

    # 进行VHACD划分
    vhacd_decomposition(name_in, name_out, name_log)
