import open3d as o3d
import numpy as np



class Registrator:

    def __init__(self, model, scene):
        self.model = model
        self.scene = scene

    def voxel_down_sample(self, voxel_size=0.02):
        """ 对模型和场景进行体素下采样 """
        self.model = self.model.voxel_down_sample(voxel_size)
        self.scene = self.scene.voxel_down_sample(voxel_size)

    def estimate_normals(self, radius=0.1, max_nn=30):
        """ 估算法线 """
        self.model.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=radius, max_nn=max_nn))
        self.scene.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=radius, max_nn=max_nn))

    def register_icp(self, threshold=0.02):
        """ 使用迭代最近点算法进行配准 """
        result = o3d.pipelines.registration.registration_icp(
            self.model, self.scene, threshold, np.eye(4),
            o3d.pipelines.registration.TransformationEstimationPointToPlane())
        return result.transformation

    def remove_noise(self, nb_points=20, radius=0.05):
        """ 基于统计分析的去噪方法 """
        cl, ind = self.model.remove_radius_outlier(nb_points, radius)
        self.model = self.model.select_by_index(ind)
        cl, ind = self.scene.remove_radius_outlier(nb_points, radius)
        self.scene = self.scene.select_by_index(ind)

    def display(self):
        """ 显示模型和场景 """
        o3d.visualization.draw_geometries([self.model, self.scene])

    def merge_point_clouds(self, pcd1, pcd2, pcd3, voxel_size=0.05, nb_neighbors=10, std_ratio=1.0):
        """ 合并三个点云，并移除独立部分 """
        # 先将三个点云合并
        merged_pcd = pcd1 + pcd2 + pcd3

        # 下采样
        merged_pcd = merged_pcd.voxel_down_sample(voxel_size)

        # 估算法线，为了后续的邻域分析
        merged_pcd.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 2, max_nn=30))

        # 计算每个点的密度，低密度的点被认为是独立部分
        densities = o3d.geometry.compute_point_cloud_density(merged_pcd)
        density_array = np.asarray(densities)

        # 使用平均值和标准差来确定密度阈值
        density_threshold = np.mean(density_array) + std_ratio * np.std(density_array)

        # 选择密度大于阈值的点
        inliers = density_array > density_threshold
        filtered_pcd = merged_pcd.select_by_index(np.where(inliers)[0])

        return filtered_pcd


if __name__ == '__main__':


    pass