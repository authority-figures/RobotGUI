import sys
import time

from PyQt5.QtWidgets import QApplication, QMainWindow
from main_window import Ui_MainWindow  # 导入从ui文件转换得到的类
from main_QT_ADS import Digital_Twin
from PyQt5.QtCore import QThread
from PyQt5.QtCore import pyqtSignal
from PyQt5.QtCore import Qt
import threading

class SimulationThread(QThread):
    '''
    用于线程保护，例如在pyqt5中开启一个gui，通过它打开一个pybullet的gui界面中出现死循环时，会导致mainwindow卡死
    在pybullet 对象的函数中套上SimulationThread可以防止这种情况发生
    '''
    def __init__(self, func, *args, **kwargs):
        super().__init__()
        self.func = func
        self.args = args
        self.kwargs = kwargs

    def run(self):
        self.func(*self.args, **self.kwargs)  # 在新线程中执行指定的函数

class MainWindow(QMainWindow, Ui_MainWindow):


    def __init__(self):
        super(MainWindow, self).__init__()
        self.setupUi(self)  # 初始化界面
        # self.pushButton.clicked.connect(self.run_my_function)
        self.DT = Digital_Twin()
        self.creat_connects()




        self.simulation_thread = SimulationThread(self.DT)  # 创建一个QThread对象
        # 你可以在这里添加更多的初始化代码，比如信号与槽的连接


    def creat_connects(self):
        self.pushButton.clicked.connect(self.run_my_function)
        self.pushButton_2.clicked.connect(self.shut_down_my_function)
        self.pushButton_3.clicked.connect(self.on_go_to_point_clicked)
        self.pushButton_4.clicked.connect(self.on_create_pybullet_env_clicked)
        self.pushButton_5.clicked.connect(self.on_load_robot_clicked)
        self.update_robot_states_check_box.stateChanged.connect(self.on_update_robot_states_changed)
        self.is_robot_states_updating = False  # 追踪复选框状态的属性



        pass

    def start_thread(self, func, *args, **kwargs):
        # 开启线程保护
        self.thread = SimulationThread(func, *args, **kwargs)
        self.thread.start()

    def on_create_pybullet_env_clicked(self):
        self.DT.creat_client(name="GUI")
        # 连接信号到槽
        # self.DT.env_list["GUI"].signal_emitter.joint_states_updated.connect(self.update_joint_states_label)
        self.DT.env_list["GUI"].signal_emitter.robot_messages_updated.connect(self.update_robot_states_labels)
        pass

    def run_my_function(self):
        # self.DT.run()
        self.start_thread(self.DT.run)
        # self.start_thread(self.show_joint_angles)
        # self.show_joint_angles()

        pass

    def on_load_robot_clicked(self):
        self.DT.load_robot()
        pass

    def shut_down_my_function(self):
        self.start_thread(self.DT.shut_down)
        pass

    def on_go_to_point_clicked(self):
        # [0.5,0.5,0.5]
        point_text = self.lineEdit.text().strip("[]")
        values = point_text.split(",")
        point_values = [float(value) for value in values]
        command = lambda: self.DT.go_to_point(pos=point_values, ori=[])
        # 将命令添加到PybulletQtEnv的命令队列中
        self.DT.env_list["GUI"].add_command(command)

        pass

    def on_update_robot_states_changed(self, state):
        # 判断是否更新显示机器人状态
        if state == Qt.Checked:
            self.DT.set_send_messages(True)
            self.is_robot_states_updating = True
        else:
            if self.is_robot_states_updating:  # 只有在状态实际改变时才执行
                self.DT.set_send_messages(False)
                self.display_none_in_labels()
                self.is_robot_states_updating = False



    def update_robot_states_labels(self, robot_messages):
        joint_states = robot_messages.joint_states
        text_joints = ', '.join([f"{joint:.2f}" for joint in joint_states])
        pos, ori = robot_messages.end_effector_pos_and_ori[0], robot_messages.end_effector_pos_and_ori[1]
        text_pos = ', '.join([f"{_:.2f}" for _ in pos])
        text_ori = ', '.join([f"{_:.2f}" for _ in ori])
        self.label_joint_angles.setText(text_joints)
        self.label_end_effector_pos_label.setText(text_pos)
        self.label_end_effector_ori_label.setText(text_ori)
        pass

    def display_none_in_labels(self):
        # 当更新机器人状态的复选框没有打钩时，清楚label的所有显示并重置
        self.label_joint_angles.setText("None")
        self.label_end_effector_pos_label.setText("None")
        self.label_end_effector_ori_label.setText("None")
        pass


def main():
    app = QApplication(sys.argv)
    mainWindow = MainWindow()
    mainWindow.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
