class RobotNotFoundException(Exception):
    """自定义异常，用于未能找到指定ID的Robot对象时抛出。"""
    def __init__(self, robot_id):
        message = f"Robot with ID {robot_id} not found in the robots list."
        super().__init__(message)