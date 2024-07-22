# %%
import open3d as o3d
import numpy as np
from file_utils import *

# %%

class PointCloud:
    def __init__(self):
        self.pcd_dict : dict[str:o3d.geometry.PointCloud()] = {}
        self.vis = o3d.visualization.Visualizer()
        self.vis.create_window()
        self.geometries = []


    def load_point_cloud(self, name='undefined', file_path=''):
        """加载点云文件"""
        if os.path.exists(file_path):
            self.pcd_dict[name] = o3d.io.read_point_cloud(file_path)
        else:
            print('文件路径不存在')
        if not self.pcd_dict[name]:
            print("Failed to load point cloud.")
        else:
            print("Point cloud loaded successfully.")

    def load_point_cloud_withGEO(self, name='undefined', pcd=None):
        """加载点云文件"""
        if pcd is not None:
            self.pcd_dict[name] = pcd
        else:
            print('点云不存在')
        if not self.pcd_dict[name]:
            print("Failed to load point cloud.")
        else:
            print("Point cloud loaded successfully.")

    def display_point_cloud(self,pcd=None):
        """显示点云"""
        if pcd is not None:
            o3d.visualization.draw_geometries(pcd)

    def show_vis(self):
        """使用Visualizer类显示点云"""
        # self.vis.create_window()

        self.vis.run()
        self.vis.destroy_window()
        return self.vis

    def init_vis_para(self,lookat_list=[0, 0, 600],up_list=[0, -1, 0],front_list=[0, 0, -1]):
        # 初始化视角参数
        ctr = self.vis.get_view_control()

        lookat = np.array(lookat_list, dtype=np.float64).reshape(3, 1)
        up = np.array(up_list, dtype=np.float64).reshape(3, 1)
        front = np.array(front_list, dtype=np.float64).reshape(3, 1)
        ctr.set_front(front)  # 点云朝向相机的方向
        ctr.set_lookat(lookat)  # 视点中心
        ctr.set_up(up)  # 上方向
        ctr.set_zoom(1)  # 缩放
        pass

    def show_normals(self):
        opt = self.vis.get_render_option()
        opt.show_coordinate_frame = True
        opt.point_size = 5
        # opt.show_point_cloud_normals = True  # This is the crucial line to show normals



        pass

    def add_point_cloud(self, name='undefined'):
        """显示点云并动态添加更多几何体"""
        self.vis.add_geometry(self.pcd_dict[name])
        self.geometries.append(self.pcd_dict[name])

    def add_geos(self, geometries):
        # 直接添加点云数据
        """显示点云并动态添加更多几何体"""
        if self.vis is not None:
            for geometry in geometries:
                self.vis.add_geometry(geometry)
                self.geometries.append(geometry)

    def save_geometries(self, file_prefix='geos',output_directory='./data', extension='ply'):
        """将当前视图中的所有几何体保存到一个点云文件"""
        all_points = []
        all_colors = []

        for geometry in self.geometries:
            if isinstance(geometry, o3d.geometry.PointCloud):
                all_points.append(np.asarray(geometry.points))
                if np.asarray(geometry.colors).size != 0:
                    all_colors.append(np.asarray(geometry.colors))

        # 合并所有点云的点和颜色
        all_points = np.concatenate(all_points, axis=0)
        if all_colors:
            all_colors = np.concatenate(all_colors, axis=0)
            final_pcd = o3d.geometry.PointCloud()
            final_pcd.points = o3d.utility.Vector3dVector(all_points)
            final_pcd.colors = o3d.utility.Vector3dVector(all_colors)
        else:
            final_pcd = o3d.geometry.PointCloud()
            final_pcd.points = o3d.utility.Vector3dVector(all_points)

        # 保存到文件
        save_PointCloud(final_pcd, file_prefix=file_prefix,output_directory=output_directory, extension=extension)


    def set_uniform_color(self, name='undefined',color=[0, 0, 0]):
        """设置点云的统一颜色"""
        self.pcd_dict[name].colors = o3d.utility.Vector3dVector(np.full((len(self.pcd_dict[name].points), 3), color))
        # self.pcd_dict[name].paint_uniform_color(color)

    def translate(self,name='undefined', translation=[0.,0.,0.], relative=True):
        """平移点云"""
        self.pcd_dict[name].translate(translation, relative)

    def rotate(self,name='undefined', rotation_matrix=[], center=(0, 0, 0)):
        """旋转点云"""
        self.pcd_dict[name].rotate(rotation_matrix, center)

    def apply_transformation(self,name='undefined', transformation_matrix=[]):
        """应用仿射变换"""
        self.pcd_dict[name].transform(transformation_matrix)

    def create_camera_box(self,size=0.1,origin=[0,0,0]):
        """创建一个简单的线框模型来表示相机的位置和朝向"""
        box_width = 0.1
        box_height = 0.02
        box_depth = 0.04
        mesh_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=size, origin=[0, 0, 0])
        box = o3d.geometry.TriangleMesh.create_box(width=box_width, height=box_height, depth=box_depth)
        box.translate(np.array([-box_width / 2, -box_height / 2, -box_depth]))  # 将箱子移动到坐标框附近以示意相机位置
        box.paint_uniform_color([0.9, 0.1, 0.1])  # 给箱子上色以区分
        self.add_geos([box,mesh_frame])
        return box, mesh_frame

    def save_point_cloud(self,name='undefined',output_directory=r'F:\python\RobotGUI_2.1\User_Defined_Demos\jig\test_point_cloud\data',file_prefix='cad_point_cloud', extension='ply'):
        save_PointCloud(self.pcd_dict[name],output_directory=output_directory, file_prefix=file_prefix, extension=extension)
        pass

    def simulate_broken_point_cloud(self,name='undefined', view_direction='x', threshold=0.5):
        """
        模拟从特定方向观看的破碎点云，并保留对应点的法线信息。

        Args:
            name (str): 点云在 self.pcd_dict 中的键名。
            view_direction (str): 观察方向（'x', 'y', 或 'z'）。
            threshold (float): 视角遮挡阈值，决定了从视角方向看时哪些点被保留。

        Returns:
            open3d.geometry.PointCloud: 处理后的点云对象。
        """
        # 加载点云对象
        pcd = self.pcd_dict[name]

        # 转换为numpy数组
        points = np.asarray(pcd.points)
        normals = np.asarray(pcd.normals)

        # 确定视角方向
        if view_direction == 'x':
            dim = 0
        elif view_direction == 'y':
            dim = 1
        elif view_direction == 'z':
            dim = 2
        else:
            raise ValueError("view_direction must be 'x', 'y', or 'z'")

        # 根据阈值过滤点
        mask = points[:, dim] >= np.percentile(points[:, dim], threshold * 100)
        filtered_points = points[mask]
        filtered_normals = normals[mask]

        # 创建新的点云对象
        new_pcd = o3d.geometry.PointCloud()
        new_pcd.points = o3d.utility.Vector3dVector(filtered_points)
        new_pcd.normals = o3d.utility.Vector3dVector(filtered_normals)

        return new_pcd

    def add_normals_to_point_cloud(self,name,radius=5., max_nn=30):
        """
        为点云添加法线信息。

        Args:
        - point_cloud_path (str): 输入点云文件的路径。
        - save_path (str, optional): 带法线的点云文件的保存路径。如果为 None，则不保存文件。

        Returns:
        - o3d.geometry.PointCloud: 带有法线的点云对象。
        """
        # 读取点云文件
        pcd = self.pcd_dict[name]

        # 计算点云的法线
        pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=5, max_nn=30))

        camera_location = np.array([0, 0, 0], dtype=np.float64)  # 以原点为视点
        # 可选：调整法线的方向
        # pcd.orient_normals_consistent_tangent_plane(k=30)
        pcd.orient_normals_towards_camera_location(camera_location)

        scale_factor = 0.05  # 可以调整这个比例因子来控制法线的显示长度
        pcd.normals = o3d.utility.Vector3dVector(np.asarray(pcd.normals) * scale_factor)

        self.pcd_dict[name] = pcd
        # # 可选：可视化点云和法线
        # o3d.visualization.draw_geometries([pcd], point_show_normal=True)
        # return pcd

    def register_point_clouds(self, source, target, voxel_size=0.001):
        # Downsample and estimate normals
        source_down = source.voxel_down_sample(voxel_size)
        target_down = target.voxel_down_sample(voxel_size)
        source_down.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 2, max_nn=30))
        target_down.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 2, max_nn=30))

        # Compute FPFH features
        source_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
            source_down,
            o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 5, max_nn=100))
        target_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
            target_down,
            o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 5, max_nn=100))

        # Perform rough registration using feature matching and RANSAC
        result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
            source_down, target_down, source_fpfh, target_fpfh, True,
            distance_threshold=voxel_size * 1.5,
            estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
            ransac_n=4,
            checkers=[o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
                      o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(voxel_size * 1.5)],
            criteria=o3d.pipelines.registration.RANSACConvergenceCriteria(4000000, 500))

        # Refine registration with ICP
        refined_result = o3d.pipelines.registration.registration_icp(
            source, target, voxel_size, result.transformation,
            o3d.pipelines.registration.TransformationEstimationPointToPlane())

        return refined_result

    def remove_background_points(self, pcd_background, pcd_scene_with_objects, distance_threshold=0.05, num_neighbors=5):
        """
        从点云B中移除点云A中的点，使用指定的移除距离d。

        参数:
        pcd_background (open3d.geometry.PointCloud): 背景点云A。
        pcd_scene_with_objects (open3d.geometry.PointCloud): 场景点云B。
        distance_threshold (float): 移除距离阈值d。

        返回:
        open3d.geometry.PointCloud: 移除背景点后的点云。
        """


        # 计算差异
        # 创建kd树用于快速邻域搜索
        pcd_background_tree = o3d.geometry.KDTreeFlann(pcd_background)

        # 定义一个新的点云存储减去背景后的点云
        filtered_points = []

        for point in pcd_scene_with_objects.points:
            [_, idx, dists] = pcd_background_tree.search_knn_vector_3d(point, num_neighbors)
            # 使用多个邻近点的距离判断
            if min(dists) > distance_threshold ** 2:  # 距离是平方值
                filtered_points.append(point)

        # 创建新的点云对象
        pcd_filtered = o3d.geometry.PointCloud()
        pcd_filtered.points = o3d.utility.Vector3dVector(np.array(filtered_points))

        return pcd_filtered

# %%

def test_ppf():
    # 创建PointCloud对象
    pc = PointCloud()
    # 加载点云
    pc.load_point_cloud(name='source', file_path="./cad_pcds/cylinder1.ply")
    pc.load_point_cloud(name='target', file_path="./cad_pcds/box1.ply")
    pc.load_point_cloud(name='scene', file_path="./data/output_colored_point_cloud_standard_with_norm0.ply")
    # pc.translate(name='source',translation=[0, 0.05, 0.05])
    rot_mat = [  [1.0000000,  0.0000000,  0.0000000],
   [0.0000000,  0.0007963, -0.9999997],
   [0.0000000,  0.9999997,  0.0007963 ],]
    pc.rotate(name='source',rotation_matrix=rot_mat)
    mesh_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=[0, 0, 0])
    pc.add_geos([mesh_frame])
    pc.add_point_cloud('source')
    # pc.add_point_cloud('target')
    pc.add_point_cloud('scene')
    pc.show_vis()

    # 获取变换矩阵
    # transformation = ppf_match.get_final_transformation()

    # mat = [[-0.9986745824516291, 0.01762778094105794, -0.04835638224845138, 98.73766250933535],
    #        [-0.0006735572053558192, -0.94391646091899, -0.3301836778625815, -2.441180459619288],
    #        [-0.05146479073847837, -0.3297134758320686, 0.9426772507963584, 2.25850167379378],
    #        [0, 0, 0, 1]]
    # mat2 = [[-0.8402199364622636, 0.5407913361105783, 0.039688652774881, 91.91641026492832],
    #          [0.2954721813477991, 0.5179839282195586, -0.8027383385361688, 95.82500594612917],
    #          [-0.4546720229142353, -0.6627498629905331, -0.5950092189916946, 263.029468150715],
    #          [0, 0, 0, 1]]
    # pc.apply_transformation(name='model',transformation_matrix=mat2)
    #
    # pc.set_uniform_color(name='model', color=[0, 0, 0])
    #
    # pc.add_point_cloud('model')
    #
    # pc.add_point_cloud('scene')
    #
    #
    # pc.create_camera_box(size=10)
    # pc.show_vis()

    pass

def create_scene():
    # 创建PointCloud对象
    pc = PointCloud()
    pc.load_point_cloud(file_path="point_cloud_data/geos0.ply")
    pc.translate(translation=[0,1,1])
    # mat =  [[-0.4119632477760986, 0.1755897799691749, -0.8941222017443399],
    #          [0.1966366701269371, -0.9410052463684224, -0.2753963439635611],
    #          [-0.8897304661767853, -0.2892703847224645, 0.3531321878268229],]
    # pc.rotate(rotation_matrix=mat)
    pc.add_point_cloud()
    pc.create_camera_box(size=10)
    pc.show_vis()
    pc.save_point_cloud(file_prefix='scene_b')
    pass

def test_broken():
    pc = PointCloud()
    pc.load_point_cloud(name='model', file_path="point_cloud_data/cad_point_cloud0.ply")
    new_pcd = pc.simulate_broken_point_cloud(name='model',view_direction='y',threshold=0.7)
    pc.load_point_cloud_withGEO(name='broken',pcd=new_pcd)
    pc.translate(name='broken',translation=[0, 10, 10])

    pc.add_point_cloud('broken')
    pc.create_camera_box(size=10)
    pc.show_vis()
    pc.save_point_cloud(name='broken', file_prefix='broken', extension='ply')
    pass

def put_model_in_scene():
    pc = PointCloud()
    pc.load_point_cloud(name='model', file_path="point_cloud_data/cad_point_cloud0.ply")
    pc.load_point_cloud(name='scene', file_path="point_cloud_data/output_colored_point_cloud_standard.ply")
    pc.translate(name='model', translation=[0, 120, 200])
    pc.add_point_cloud('model')
    pc.add_point_cloud('scene')
    pc.show_vis()
    pc.save_geometries()


    pass

def norm():
    pc = PointCloud()
    pc.load_point_cloud(name='scene', file_path="./data/combined_3pc0.ply")
    pc.add_normals_to_point_cloud(name='scene',radius=0.005, max_nn=30)
    pc.add_point_cloud('scene')

    o3d.visualization.draw_geometries([pc.pcd_dict['scene']],point_show_normal=True)
    pc.save_point_cloud(name='scene',file_prefix='combined_3pc0_with_norm',extension='ply')
    # pc.show_normals()
    # pc.show_vis()
    pass

def show_ply():
    pc = PointCloud()
    path1 = 'point_cloud_data/realsence/test1.ply'
    path2 = 'point_cloud_data/jig/Box2_11.72.ply'
    path3 = 'point_cloud_data/jig/Box2_11.73.ply'
    path4 = 'point_cloud_data/jig/Box2_11.74.ply'
    path5 = 'point_cloud_data/jig/Box2_11.75.ply'
    pc.load_point_cloud(name='scene1', file_path=path2)
    pc.load_point_cloud(name='scene2', file_path=path3)
    pc.load_point_cloud(name='scene3', file_path=path4)
    pc.load_point_cloud(name='scene4', file_path=path5)
    pc.translate(name='scene2', translation=[0.117,0,0])
    pc.translate(name='scene3', translation=[0, -0.051, 0])
    pc.translate(name='scene4', translation=[0.117, -0.051, 0])
    pc.set_uniform_color('scene1',color=[0.8,0,0])
    pc.set_uniform_color('scene2', color=[0, 0, 0.8])
    pc.set_uniform_color('scene3', color=[0.8, 0.5, 0])
    pc.set_uniform_color('scene4', color=[0, 0.5, 0.8])
    pc.add_point_cloud('scene1')
    pc.add_point_cloud('scene2')
    pc.add_point_cloud('scene3')
    pc.add_point_cloud('scene4')
    pc.create_camera_box()
    pc.show_vis()
    pc.save_geometries(file_prefix='combined_4pc', extension='ply')
    pass


def show_my_ply():
    pc = PointCloud()
    scene = r'F:\python\RobotGUI_2.1\User_Defined_Demos\jig\test_point_cloud\data\combined_3pc0.ply'
    scene1 = r'F:\python\RobotGUI_2.1\User_Defined_Demos\jig\test_point_cloud\data\objs_with_table_empty0.ply'
    scene2 = r'F:\python\RobotGUI_2.1\User_Defined_Demos\jig\test_point_cloud\data\objs_with_table_empty1.ply'
    scene3 = r'F:\python\RobotGUI_2.1\User_Defined_Demos\jig\test_point_cloud\data\objs_with_table_empty2.ply'
    pc.load_point_cloud(name='scene1', file_path=scene1)
    pc.load_point_cloud(name='scene2', file_path=scene2)
    pc.load_point_cloud(name='scene3', file_path=scene3)
    # # pc.set_uniform_color(name='scene1',color=[0,0,1])
    # # pc.set_uniform_color(name='scene2', color=[0, 1, 0])
    # # pc.set_uniform_color(name='scene3', color=[1, 0, 0])
    # mesh_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=[0, 0, 0])
    # # pc.add_normals_to_point_cloud(name='scene1')
    o3d.visualization.draw_geometries([pc.pcd_dict['scene1'],pc.pcd_dict['scene2'],pc.pcd_dict['scene3']], point_show_normal=True)
    # pc.add_point_cloud('scene1')
    # pc.add_point_cloud('scene2')
    # pc.add_point_cloud('scene3')
    pc.save_geometries(file_prefix='combined_3pc_empty', extension='ply')
    # pc.load_point_cloud(name='scene', file_path=scene)
    # voxel_size = 0.001  # 定义体素大小
    # pcd_voxel_down = pc.pcd_dict['scene'].voxel_down_sample(voxel_size)
    # o3d.visualization.draw_geometries([pcd_voxel_down],point_show_normal=True)
    pass

def test_register():
    pc = PointCloud()
    source_cloud = o3d.io.read_point_cloud("./cad_pcds/box1.ply")  # model
    target_cloud = o3d.io.read_point_cloud("./data/combined_3pc0_with_norm0.ply")  # scene

    # Register point clouds
    result = pc.register_point_clouds(source_cloud, target_cloud)
    print("Transformation matrix:")
    print(result.transformation)
    pass

def test_remove_points():
    pc = PointCloud()
    background = r'F:\python\RobotGUI_2.1\User_Defined_Demos\jig\test_point_cloud\data\combined_3pc_empty0.ply'
    scene = r'F:\python\RobotGUI_2.1\User_Defined_Demos\jig\test_point_cloud\data\combined_3pc0.ply'
    pc.load_point_cloud(name='background', file_path=background)
    pc.load_point_cloud(name='scene', file_path=scene)
    pcd_background = o3d.io.read_point_cloud(background)
    pcd_scene_with_objects = o3d.io.read_point_cloud(scene)

    pc.set_uniform_color(name='background',color=[0,0,1])
    pc.set_uniform_color(name='scene', color=[0, 1, 0])
    # o3d.visualization.draw_geometries([pc.pcd_dict['background'],pc.pcd_dict['scene']])
    distance_threshold = 0.001
    pcd_filtered = pc.remove_background_points(pcd_background, pcd_scene_with_objects, distance_threshold,num_neighbors=1)
    o3d.visualization.draw_geometries([pcd_filtered,pc.pcd_dict['scene']])
    pass


if __name__ == "__main__":
    # create_scene()
    # test_ppf()
    # test_broken()
    # put_model_in_scene()
    # norm()
    # show_ply()
    # show_my_ply()
    # test_register()
    test_remove_points()
    pass