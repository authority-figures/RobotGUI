import pybullet as p
import pybullet_data as pd
import os
import glob
import pymeshlab

def convert_stl_to_obj(stl_file, obj_file):
    ms = pymeshlab.MeshSet()
    ms.load_new_mesh(stl_file)
    ms.save_current_mesh(obj_file)


def find_files(directories, file_type):
    # 确保路径以斜杠结束
    file_names = []
    for directory in directories:
        directory_path = directory
        directory = os.path.join(directory, '')
        # 搜索指定后缀的文件
        if file_type == 'stl' or file_type == 'STL':
            files = glob.glob(directory + '*.stl') + glob.glob(directory + '*.STL')
        if file_type == 'obj':
            files = glob.glob(directory + '*.obj')
        # 获取文件名（不包括路径）
        file_names.extend([directory_path+os.path.basename(file) for file in files])
    return file_names


def vhacd_decomposition(name_in, name_out, name_log, resolution=1000000, depth=1024, concavity=0.0001):
    params = {
        'resolution': resolution,
        'depth': depth,
        'concavity': concavity,
        'planeDownsampling': 4,
        'convexhullDownsampling': 4,
        'alpha': 0.05,
        'beta': 0.05,
        'gamma': 0.001,
        'pca': 0,
        'mode': 0,  # 0=voxel based, 1=tetrahedron based
        'maxNumVerticesPerCH': 256,
        'minVolumePerCH': 0.00001,
    }
    p.vhacd(name_in, name_out, name_log,)# **params)


if __name__ == "__main__":
    save_path="F:/sw/urdf_files/work_piece1.SLDASM/"
    # 替换为您的目标文件夹路径
    fold_path = ["F:/sw/urdf_files/work_piece1.SLDASM/meshes/"]
    stl_file_list = find_files(fold_path, "stl")
    print("找到的STL文件列表:", stl_file_list)

    for stl_file in stl_file_list:
        stl_file_path = stl_file
        obj_file_path = save_path + "processed-geoms/"+stl_file.split("/")[-1].split(".")[0]+'.obj'
        convert_stl_to_obj(stl_file_path, obj_file_path)

    # p.connect(p.DIRECT)
    # obj_file_list = find_files([save_path + "processed-geoms/"], "obj")
    # for obj_file in obj_file_list:
    #     name_in = obj_file
    #     name_out = obj_file.split(".obj")[0]+'_vhacd.obj'
    #     name_log = "log.txt"
    #     vhacd_decomposition(name_in, name_out, name_log)

    # p.connect(p.DIRECT)
    # p.vhacd("Geometry/endeffect_tool.obj", "Geometry/endeffect_tool_vhacd.obj", "log.txt")