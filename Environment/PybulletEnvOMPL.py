try:
    from ompl import util as ou
    from ompl import base as ob
    from ompl import geometric as og
except ImportError:
    raise ImportError("OMPL is not found !")
from utils import *
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ''))
from PybulletEnv import PybulletEnv
from RobotDir.Robot import Robot
INTERPOLATE_NUM = 500
DEFAULT_PLANNING_TIME = 100


class RobotOMPL(Robot):
    pass


class PbStateSpace(ob.RealVectorStateSpace):
    def __init__(self, num_dim) -> None:
        super().__init__(num_dim)
        self.num_dim = num_dim
        self.state_sampler = None

    def allocStateSampler(self):
        """
        This will be called by the internal OMPL planner
        """
        # WARN: This will cause problems if the underlying planner is multi-threaded!!!
        if self.state_sampler:
            return self.state_sampler

        # when ompl planner calls this, we will return our sampler
        return self.allocDefaultStateSampler()

    def set_state_sampler(self, state_sampler):
        """
        Optional, Set custom state sampler.
        """
        self.state_sampler = state_sampler


class PybulletEnvOMPL(PybulletEnv):
    pb_space = None
    ss = None
    si = None
    planner = None

    def add_obstacle(self, robot: RobotOMPL):
        self.obstacle = robot

    def set_robot(self, robot: RobotOMPL):
        self.robot = robot
        self.robot_id = robot.id_robot

    def build_state_space(self):
        self.pb_space = PbStateSpace(self.robot.num_avail_joints)
        bounds = ob.RealVectorBounds(self.robot.num_avail_joints)
        for i, info in enumerate([self.robot.info_joints[j] for j in self.robot.ids_avail_joints]):
            # print(info[0])
            bounds.setLow(i, float(info[1]))
            bounds.setHigh(i, float(info[2]))
            # print(info[0], ":", info[1], ",", info[2])
        self.pb_space.setBounds(bounds)

        self.ss = og.SimpleSetup(self.pb_space)
        self.si = self.ss.getSpaceInformation()
        self.si.setStateValidityChecker(ob.StateValidityCheckerFn(self.is_state_valid))

    def is_state_valid(self, state):
        self.robot.set_joints_states(state)
        points = self.get_collision_points()
        if points[0]:
            return False
        return True

    def set_planner(self, planner_name):
        if planner_name == "PRM":
            self.planner = og.PRM(self.si)
        elif planner_name == "RRT":
            self.planner = og.RRT(self.si)
        elif planner_name == "RRTConnect":
            self.planner = og.RRTConnect(self.si)
        elif planner_name == "RRTstar":
            self.planner = og.RRTstar(self.si)
        elif planner_name == "EST":
            self.planner = og.EST(self.si)
        elif planner_name == "FMT":
            self.planner = og.FMT(self.si)
        elif planner_name == "BITstar":
            self.planner = og.BITstar(self.si)
        else:
            print("{} not recognized, please add it first".format(planner_name))
            return

    def plan_from_start_goal(self, start, goal, planner_name="BITstar", allowed_time=DEFAULT_PLANNING_TIME):
        print("start_planning")

        orig_robot_state = self.robot.get_joints_states(mode=0)
        # set the start and goal states;
        s = ob.State(self.pb_space)
        g = ob.State(self.pb_space)
        for i in range(len(start)):
            s[i] = start[i]
            g[i] = goal[i]

        pdef = ob.ProblemDefinition(self.si)
        pdef.setStartAndGoalStates(s, g)
        self.set_planner(planner_name)
        self.planner.setProblemDefinition(pdef)
        self.planner.setup()

        print(self.si.settings())
        print(pdef)
        # attempt to solve the problem within allowed planning time
        solved = self.planner.solve(allowed_time)
        res = False
        sol_path_list = []
        if solved:
            print("Found solution: interpolating into {} segments".format(INTERPOLATE_NUM))
            # print the path to screen
            sol_path_geometric = pdef.getSolutionPath()
            sol_path_geometric.interpolate(INTERPOLATE_NUM)
            sol_path_states = sol_path_geometric.getStates()
            sol_path_list = [self.state_to_list(state) for state in sol_path_states]
            # print(len(sol_path_list))
            # print(sol_path_list)
            for sol_path in sol_path_list:
                self.is_state_valid(sol_path)
            res = True
        else:
            print("No solution found")

        # reset Robot state
        self.robot.set_joints_states(orig_robot_state)
        return res, sol_path_list

    def state_to_list(self, state):
        return [state[i] for i in range(self.robot.num_avail_joints)]

