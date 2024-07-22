from pybullet_utils import bullet_client as bc
from pybullet_utils import urdfEditor as ed
import pybullet
import pybullet_data
import time
import pybullet as p


def print_joint_states(id_robot, client,jointIndex=None):
    """
    打印当前所有关节的信息。
    """

    num_all_joints = p.getNumJoints(id_robot,physicsClientId=client)

    print("+" * 50)
    for i in range(num_all_joints):
        if jointIndex is not None:
            i = jointIndex
        joint_info = p.getJointInfo(id_robot, i,physicsClientId=client)
        joint_state = p.getJointState(id_robot, i,physicsClientId=client)
        link_state = p.getLinkState(id_robot, i,physicsClientId=client)
        joint_name = joint_info[1].decode('utf-8')
        joint_angle = joint_state[0]
        joint_velocity = joint_state[1]
        joint_reaction_forces = joint_state[2]
        joint_torque = joint_state[3]
        joint_world_position = link_state[4]
        joint_world_orientation = link_state[5]


        joint_lower_limit, joint_upper_limit = joint_info[8], joint_info[9]
        print(f"Joint {i} - {joint_name}:")
        print(f"  Angle: {joint_angle}")
        print(f"  Dynamic Range: [{joint_lower_limit}, {joint_upper_limit}]")
        print(f"  Velocity: {joint_velocity}")
        print(f"  Reaction Forces: {joint_reaction_forces}")
        print(f"  Torque: {joint_torque}")
        print(f"  World Position: {joint_world_position}")
        print(f"  World Orientation (quaternion): {joint_world_orientation}")
        if jointIndex is not None:
            break

        print()

    print("-" * 50)

p0 = bc.BulletClient(connection_mode=pybullet.DIRECT)
p0.setAdditionalSearchPath(pybullet_data.getDataPath())

p1 = bc.BulletClient(connection_mode=pybullet.DIRECT)
p1.setAdditionalSearchPath(pybullet_data.getDataPath())

#can also connect using different modes, GUI, SHARED_MEMORY, TCP, UDP, SHARED_MEMORY_SERVER, GUI_SERVER

husky = p1.loadURDF("husky/husky.urdf", flags=p0.URDF_USE_IMPLICIT_CYLINDER)
kuka = p0.loadURDF("kuka_iiwa/model.urdf")

ed0 = ed.UrdfEditor()
ed0.initializeFromBulletBody(husky, p1._client)
ed1 = ed.UrdfEditor()
ed1.initializeFromBulletBody(kuka, p0._client)
#ed1.saveUrdf("combined.urdf")

parentLinkIndex = 0

jointPivotXYZInParent = [0, 0, 0]
jointPivotRPYInParent = [0, 0, 0]

jointPivotXYZInChild = [0, 0, 0]
jointPivotRPYInChild = [0, 0, 0]

newjoint = ed0.joinUrdf(ed1, parentLinkIndex, jointPivotXYZInParent, jointPivotRPYInParent,
                        jointPivotXYZInChild, jointPivotRPYInChild, p0._client, p1._client)
newjoint.joint_type = p0.JOINT_FIXED

ed0.saveUrdf("combined.urdf")

print(p0._client)
print(p1._client)
print("p0.getNumBodies()=", p0.getNumBodies())
print("p1.getNumBodies()=", p1.getNumBodies())

pgui = bc.BulletClient(connection_mode=pybullet.GUI)
pgui.configureDebugVisualizer(pgui.COV_ENABLE_RENDERING, 0)

orn = [0, 0, 0, 1]
# ed0.createMultiBody([0, 0, 0], orn, pgui._client,)
pgui.loadURDF('combined.urdf',basePosition=[0,0,0],useFixedBase=True)
pgui.setRealTimeSimulation(1)

pgui.configureDebugVisualizer(pgui.COV_ENABLE_RENDERING, 1)


print_joint_states(0,pgui._client)
info = pgui.getJointInfo(0, 0)
print(info)
pgui.setJointMotorControl2(0,12,p.VELOCITY_CONTROL, targetVelocity=1,force=10,physicsClientId=pgui._client)
# pgui.resetJointState(0,12,1.57,physicsClientId=pgui._client)


while (pgui.isConnected()):

    pgui.getCameraImage(320, 200, renderer=pgui.ER_BULLET_HARDWARE_OPENGL)
    time.sleep(1. / 240.)
    p.stepSimulation(physicsClientId=pgui._client)
