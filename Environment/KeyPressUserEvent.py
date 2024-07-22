import pybullet as p
from utils import *
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ''))


class KeyPressUserEvent:

    def __init__(self, id_client):
        self.id_client = id_client
        self.user_input = None

        self.key_to_function = {
            ord('a'): {"func": "DisplayPath",
                       "value": 0},  # gui验证路径
        }

    def get_user_input(self, user_input):
        self.user_input = user_input

    def get_key_press(self, env):
        if self.user_input is None:
            return 0
        keys = p.getKeyboardEvents(physicsClientId=self.id_client)
        for key, state in keys.items():
            if state & p.KEY_IS_DOWN:
                if key in self.key_to_function:
                    event = self.key_to_function[key]

                    if event["func"] == "DisplayPath":
                        val = event["value"]
                        self.display_path(env, self.user_input, val)

                    elif event["func"] == "OtherFunction":
                        val = event["value"]
                        # self.other_function(env, self.user_input, val)

    # Defining user key press function
    def display_path(self, env, user_input, val):
        # user_input: [[robot_id1, robot_id2], [path1, path2]], path: list<float*6>
        for robot_id, path in zip(user_input[0], user_input[1]):
            robot = env.robots[robot_id]
            try:
                robot.set_joints_states(path[val])
                self.key_to_function[ord('a')]["value"] += 1
            except IndexError:
                print("IndexError:", self.key_to_function[ord('a')]["value"], "is out of path's index range")

