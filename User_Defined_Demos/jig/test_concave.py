import time
import pybullet as p
from utils import *
from Environment.PybulletEnv import PybulletEnv
from RobotDir.Robot import Robot
from RobotDir.Machine import Machine
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ''))

class BaseDemo:
    # 创建pybullet连接
    def __init__(self):
        id_client = p.connect(p.GUI)
        p.setGravity(0,0,-10,physicsClientId=id_client)
        p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1, shadowMapWorldSize=1, shadowMapIntensity=1, physicsClientId=id_client)
        p.setAdditionalSearchPath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '../Robot/urdf/'))
        self.env = PybulletEnv(id_client=id_client)
        # p.setPhysicsEngineParameter(contactBreakingThreshold=0.001, allowedCcdPenetration=0.0001, numSolverIterations=10)
        # p.setPhysicsEngineParameter(contactBreakingThreshold=0.02,
        #                             numSolverIterations=50)
        box_urdf = "F:\\python\\RobotGUI_2.1\\User_Defined_Demos\\jig\\urdf_files\\box.urdf"
        machine_file_name = "F:\sw\\urdf_files\c501-simple.SLDASM\\urdf\c501-simple.SLDASM.urdf"
        robot_urdf = "F:\sw\\urdf_files\i7-nofork.SLDASM\\urdf\i7-nofork.SLDASM.urdf"
        # self.env.load_robot(fileName=machine_file_name, basePosition=(0, 0, 0),
        #                     useFixedBase=1, flags=0, start=[0, 0, 0, 0, 0, 0], f_pinrt=True,Robot_class=Machine)
        self.link_c = Robot(0)
        # self.link_c.load_urdf(fileName="F:\\sw\\urdf_files\\Link_C\\urdf\\Link_C.urdf",basePosition=[0,0,0.0],useFixedBase=True)
        self.link_c.load_urdf(fileName="F:\\sw\\urdf_files\\Link_C_18parts\\urdf\\Link_C_18parts.urdf", basePosition=[0, 0, 0.0],useFixedBase=True)

        self.T_block = Robot(0)
        self.T_block.load_urdf(fileName="F:\\sw\\urdf_files\\T_block\\urdf\\T_block_new.urdf", basePosition=[0.0, 0.2,0.3],baseOrientation=[0,0,0,1],useFixedBase=0)
        p.changeDynamics(
            bodyUniqueId=self.T_block.id_robot,
            linkIndex=-1,
            lateralFriction=0.1,
            # restitution=0.1,
            # ccdSweptSphereRadius=0.01,
            mass=1,
            collisionMargin=0.0001
        )
        p.changeDynamics(
            bodyUniqueId=self.T_block.id_robot,
            linkIndex=-1,
            lateralFriction=0.1,
            # restitution=0.1,
            mass=1,
            collisionMargin=0.0001
        )
        p.changeDynamics(
            bodyUniqueId=self.link_c.id_robot,
            linkIndex=-1,
            lateralFriction=0.1,
            # restitution=0.1,
            mass=0,
            collisionMargin=0.0001
        )
        p.changeDynamics(
            bodyUniqueId=self.link_c.id_robot,
            linkIndex=12,
            lateralFriction=0.1,
            # restitution=0.1,
            mass=0,
            collisionMargin=0.0001
        )
        p.changeDynamics(
            bodyUniqueId=self.link_c.id_robot,
            linkIndex=13,
            lateralFriction=0.1,
            # restitution=0.1,
            mass=0,
            collisionMargin=0.0001
        )


        # box_id = p.loadURDF(fileName=box_urdf,basePosition=[0.0,0.0,1],useFixedBase=0)
        # 创建一个边长为50mm的立方体
        cube_half_extent = 0.013  # 50mm / 2
        cube_mass = 0  # 立方体的质量
        cube_collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=[cube_half_extent] * 3)
        cube_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[cube_half_extent] * 3, rgbaColor=[1, 1, 1, 1])
        # 创建立方体
        self.box_id = p.createMultiBody(baseMass=cube_mass, baseCollisionShapeIndex=cube_collision,
                                    baseVisualShapeIndex=cube_visual, basePosition=[0,0,0.1],)

        # balde = p.loadURDF("F:\\sw\\urdf_files\\blade_6061\\urdf\\blade_6061.urdf",
        #                    basePosition=[0, 0, 0.2],
        #                    baseOrientation=[0, 0, 0.707, 0.707], useFixedBase=0)







    def env_hold_on(self):
        p.setTimeStep(1 / 240)  # 或者更小
        # p.setPhysicsEngineParameter(contactBreakingThreshold=0.02,contactSlop=0.0001,allowedCcdPenetration=0.001, numSolverIterations=50)
        while True:

            self.env.update(mode=1, robot_mode=2)
            data = self.T_block.show_link_sys(-1,0.1,1)
            print(data)
            time.sleep(10 / 240.)


def test_lip():
    import pybullet_data
    # 启动 PyBullet
    physicsClient = p.connect(p.GUI)  # 或 p.DIRECT 用于非图形化版本
    p.setAdditionalSearchPath(pybullet_data.getDataPath())  # 设置数据路径
    # p.setPhysicsEngineParameter(contactBreakingThreshold=0.0001, allowedCcdPenetration=0.02, numSolverIterations=20)
    # 加载地面平面
    p.loadURDF("plane.urdf")

    # 设置重力
    p.setGravity(0, 0, -10)
    # 创建两个平板，两侧对称放置
    halfExtents = [0.2, 1, 0.02]  # 平板的尺寸 (x, y, z)，单位为米
    plate1 = p.createCollisionShape(p.GEOM_BOX, halfExtents=halfExtents)
    plate2 = p.createCollisionShape(p.GEOM_BOX, halfExtents=halfExtents)

    # 定义平板的位置，间隙为50mm
    plate_pos1 = [0.25, 0, 0.1]  # x位置为间隙的一半加上平板的一半厚度
    plate_pos2 = [-0.25, 0, 0.1]

    # 在物理环境中创建平板
    p.createMultiBody(baseMass=0, baseCollisionShapeIndex=plate1, basePosition=plate_pos1)
    p.createMultiBody(baseMass=0, baseCollisionShapeIndex=plate2, basePosition=plate_pos2)

    # 创建一个边长为50mm的立方体
    cube_half_extent = 0.081  # 50mm / 2
    cube_mass = 1  # 立方体的质量
    cube_collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=[cube_half_extent] * 3)
    cube_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[cube_half_extent] * 3, rgbaColor=[1, 0, 0, 1])
    cube_pos = [0.2, 0.0, 0.5]  # 从较高位置自由落下

    # 创建立方体
    cube_id = p.createMultiBody(baseMass=cube_mass, baseCollisionShapeIndex=cube_collision,
                                baseVisualShapeIndex=cube_visual, basePosition=cube_pos)

    # T_block = p.loadURDF("F:\\sw\\urdf_files\\T_block\\urdf\\T_block.urdf", basePosition=[0.2, 0.05, 0.6],
    #                           baseOrientation=[0, 0, 0, 1], useFixedBase=0)
    work_piece = Robot(0)
    work_piece.load_urdf(fileName="F:\\sw\\urdf_files\\work_piece1.SLDASM\\urdf\\work_piece1.SLDASM.urdf", basePosition=[-0.1, 0.4, 0.2],
                              baseOrientation=[0, 0, 0, 1], useFixedBase=0)
    # balde = p.loadURDF("F:\\sw\\urdf_files\\blade_6061\\urdf\\blade_6061.urdf",
    #                         basePosition=[0.2, 0, 0.6],
    #                         baseOrientation=[0, 0, 0.707, 0.707], useFixedBase=0)
    T_block = Robot(0)
    T_block.load_urdf(fileName="F:\\sw\\urdf_files\\T_block\\urdf\\T_block.urdf", basePosition=[0.2, 0.15, 0.6],
                              baseOrientation=[0, 0, 0, 1], useFixedBase=0)
    blade = Robot(0)
    blade.load_urdf(fileName="F:\\sw\\urdf_files\\blade_6061\\urdf\\blade_6061.urdf",
                            basePosition=[0.2, -0.15, 0.6],
                            baseOrientation=[0, 0, 0.707, 0.707], useFixedBase=0)

    link_c = Robot(0)
    link_c.load_urdf(fileName="F:\\sw\\urdf_files\\Link_C\\urdf\\Link_C.urdf",basePosition=[0,-0.6,0.3],baseOrientation=[ 0.9999997, 0, 0, 0.0007963 ],useFixedBase=0)

    p.changeDynamics(link_c.id_robot, -1, collisionMargin=0.0001, mass=1)
    p.changeDynamics(work_piece.id_robot, -1, collisionMargin=0.0001, mass=1)
    p.changeDynamics(blade.id_robot, -1, collisionMargin=0.0001,mass=1)
    p.changeDynamics(T_block.id_robot, -1, collisionMargin=0.0003)
    p.changeDynamics(T_block.id_robot, 0, collisionMargin=0.0003)

    # 运行仿真
    while True:
        p.stepSimulation()
        # T_block.show_link_sys(-1, 0.1, type=1)

        time.sleep(1./240.)

    # 断开连接
    p.disconnect()

if __name__ == '__main__':
    # test_lip()
    demo = BaseDemo()
    demo.env_hold_on()



