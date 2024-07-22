import sys
import time
import math
import re
import pyads
from PyQt5.QtWidgets import QApplication, QMainWindow
from UI.mainwindow import Ui_MainWindow, AutoCompleteTextEdit  # 导入从ui文件转换得到的类
from PyQt5.QtWidgets import QMainWindow, QApplication, QWidget, QVBoxLayout, QLabel,QFileDialog
from PyQt5.QtGui import QIcon,QPixmap
from PyQt5 import QtWidgets, QtCore, QtGui
from embed_window import EmbedPyBullet
from PybulletController import Pybullet_Controller
from Process_Controller import Process_Controller
from PyQt5.QtWidgets import QMessageBox
from PyQt5.QtCore import pyqtSignal, QThread, QTimer, Qt, QDir
from queue import Empty
from Process_Controller import RobotMessages,processes_info
from directEnvDialog import DirectEnvDialog
from ExecuteCmd import ExecuteCmd,Completer
from ADSCommunicator import ADSCommunicator,ADSCommunicatorThread
class DataListenerThread(QThread):
    # 创建一个信号
    data_received = pyqtSignal(RobotMessages)

    def __init__(self, data_queue=None):
        super().__init__()
        self.data_queue = data_queue
        self.running = True

    def add_data_queue(self,data_queue):
        self.data_queue = data_queue

    def run(self):

        while self.running:
            if self.data_queue is not None:
                try:
                    data = self.data_queue.get(timeout=1)  # 从队列中获取数据
                    if data is not None:
                        self.data_received.emit(data)  # 发射信号
                    else:   # 如果发送信号的env暂停，保留信息，同时改变标志isnone
                        # data.isNone = True
                        self.data_received.emit(RobotMessages(isNone=True))  # 发射信号
                        # self.data_received.emit(data)  # 发射信号
                except Empty:
                    self.data_received.emit(RobotMessages(isNone=True))  # 发射信号
                    continue
                except Exception as e:
                    print('DataListenerThread ERROR:',e)
            else:
                print('data_queue 未定义')

    def stop(self):
        self.running = False




class Stream:
    def __init__(self, text_widget):
        self.text_widget = text_widget
        self.buffer = ""  # 添加一个缓存区来存储部分行

    def write(self, message):
        # 将新消息添加到缓存区
        self.buffer += message

        # 检查是否存在完整的行
        if '\n' in self.buffer:
            # 将完整的行（可能是多行）发送到 QTextEdit
            lines = self.buffer.split('\n')

            # 最后一个元素可能是新的部分行，所以我们将其保留在缓存中
            for line in lines[:-1]:
                self.text_widget.append(line)
            self.buffer = lines[-1]  # 保留任何不完整的行

    def flush(self):
        # 如果在程序结束时还有残余数据，可以选择在这里处理
        pass


class MainWindow(QMainWindow, Ui_MainWindow):


    def __init__(self):
        super(MainWindow, self).__init__()
        self.GUI_paths_dict = {}
        self.GUI_latest_robot_messages = RobotMessages(isNone=True)
        self.setupUi(self)  # 初始化界面

        # self.pushButton.clicked.connect(self.run_my_function)
        # self.pybullet_controller = Pybullet_Controller()
        self.process_controller = Process_Controller()

        self.init_listener_threads()
        self.directEnvDialog = DirectEnvDialog()  # 实例化子窗口
        self.adsCommunicatorDialog = ADSCommunicator()
        self.ads_thread = ADSCommunicatorThread(self.adsCommunicatorDialog)
        # self.ads_thread.start()
        # self.adsCommunicatorDialog = ADSCommunicator()

        self.creat_connects()
        self.current_button = None  # 当前按钮

        self.textEdit_cmd.setReadOnly(True)  # 设置为只读，防止用户编辑
        # 重定向 print 到 QTextEdit
        sys.stdout = Stream(self.textEdit_cmd)
        self.textEdit_cmd_input.installEventFilter(self)
        self.init_cmd_input()

        # 为cmd 添加窗口实例
        self.executor = ExecuteCmd(self,)
        self.completer = Completer(self.executor.local_context)
        # self.textEdit_cmd_input.set_completer(Completer(self.executor.local_context))
        # self.executor = {'main_window': self}
        self.textEdit_cmd_input.set_completer(self.completer)

    def closeEvent(self, event):
        # 在窗口关闭时调用清理函数
        self.process_controller.stop_all_envs()
        self.directEnvDialog.close()  # 显式关闭子窗口
        self.adsCommunicatorDialog.close()
        event.accept()  # 确保窗口会被关闭


    def init_listener_threads(self):
        """初始化数据监听线程"""
        self.listener_thread_dict = {
            'GUI': DataListenerThread(None),
            'DIRECT': DataListenerThread(None),
            'DT': DataListenerThread(None),
            'DT_ENV': DataListenerThread(None),
        }

    def creat_connects(self):
        # create env
        self.actionGUI.triggered.connect(self.on_gui_environment)
        self.actionDirect.triggered.connect(self.on_direct_environment)
        self.actionDigitalTwin.triggered.connect(self.on_digital_twin_environment)
        self.actionAll.triggered.connect(self.on_start_all_environments)

        # load robot
        self.actionLoadRobot_DT.triggered.connect(self.on_load_robot)
        self.actionLoadRobot_GUI.triggered.connect(self.on_load_robot)
        self.actionLoadRobot_DIRECT.triggered.connect(self.on_load_robot)

        # Shut down env
        self.actionShutDown_GUI.triggered.connect(self.on_terminate_env)
        self.actionShutDown_DIRECT.triggered.connect(self.on_terminate_env)
        self.actionShutDown_DT.triggered.connect(self.on_terminate_env)

        # run env
        # self.pushButton_run_GUI.clicked.connect(self.on_run_GUI_env)
        self.pushButton_run_GUI.clicked.connect(self.on_run_env)
        self.pushButton_run_DIRECT.clicked.connect(self.on_run_env)
        self.pushButton_run_DT.clicked.connect(self.on_run_env)

        # copy button
        self.GUI_copy_pos_btn.clicked.connect(self.copy_data)
        self.GUI_copy_ori_btn.clicked.connect(self.copy_data)
        self.GUI_copy_joints_angle_btn.clicked.connect(self.copy_data)

        # receive data
        self.listener_thread_dict['GUI'].data_received.connect(self.update_GUI_data)
        self.listener_thread_dict['DT_ENV'].data_received.connect(self.update_DT_data)

        # set data to GUI
        self.GUI_set_pos_btn.clicked.connect(self.on_set_robot_data)
        self.GUI_set_ori_btn.clicked.connect(self.on_set_robot_data)
        self.GUI_set_joints_angle_btn.clicked.connect(self.on_set_robot_data)

        # Connect the signal to the slot
        # self.adsCommunicatorDialog.pathDataSignal.connect(self.receive_path_data)
        self.ads_thread.adsCommunicatorDialog.pathDataSignal.connect(self.receive_path_data)
        # run path_list in GUI
        self.GUI_runPath_btn.clicked.connect(self.run_GUI_path_list)
        self.GUI_reset_btn.clicked.connect(self.reset_GUI_robot_state)
        self.GUI_send_path_btn.clicked.connect(self.send_GUI_path_list)

        # DT connection
        self.connect_to_DT_btn.clicked.connect(self.on_create_connection)

        # clear|save logs
        self.clear_cmd_log_btn.clicked.connect(self.on_clear_logs)
        self.save_cmd_log_btn.clicked.connect(self.on_save_logs)

        # DIRECT dialog
        self.DIRECT_test_btn1.clicked.connect(self.openDirectEnvDialog)

        # ADS dialog
        self.control_test_btn2.clicked.connect(self.openADSDialog)
        self.adsCommunicatorDialog.ReadvariableUpdated.connect(self.handleVariableUpdate)   # 处理接收到的PLC变量
        self.GUI_send_state_btn.clicked.connect(self.send_GUI_state)

    pass

    def init_pybullet_embedding(self):
        # 创建和设置 EmbedPyBullet 的实例
        # self.embed_pybullet_widget = EmbedPyBullet(embed_type="Container")
        # self.embed_pybullet_widget = EmbedPyBullet(embed_type="")
        self.embed_pybullet_widget2GUI = EmbedPyBullet(embed_type="")
        self.embed_pybullet_widget2DT = EmbedPyBullet(embed_type="")
        # self.embed_pybullet_widget2GUI = EmbedPyBullet(win_hwnd=self.process_controller.envs['GUI']['win_hwnd'],
        #                                            win_name=self.process_controller.envs['GUI']['win_name'],
        #                                            embed_type="")
        # self.embed_pybullet_widget2DT = EmbedPyBullet(win_hwnd=self.process_controller.envs['DT']['win_hwnd'],
        #                                                win_name=self.process_controller.envs['DT']['win_name'],
        #                                                embed_type="")


        # self.verticalLayout_embed_DT.addWidget(self.embed_pybullet_widget)
        # self.verticalLayout_GUI.addWidget(self.embed_pybullet_widget)


    def on_gui_environment(self):

        try:
            print("GUI环境被创建")
            # self.pybullet_controller.creat_client(env_name="GUI", type="GUI")
            self.process_controller.create_env(env_name="GUI")
            self.embed_pybullet_widget2GUI = EmbedPyBullet(win_hwnd=self.process_controller.envs['GUI']['win_hwnd'],
                                                       win_name=self.process_controller.envs['GUI']['win_name'],
                                                       embed_type="")
            self.frame_GUI.layout().addWidget(self.embed_pybullet_widget2GUI)
            self.listener_thread_dict['GUI'].add_data_queue(self.process_controller.envs['GUI']['data_queue'])
            self.listener_thread_dict['GUI'].start()  # 开启监听
        except Exception as e:
            print("发生异常 on_gui_environment:", e)

    def on_direct_environment(self):

        try:
            print("DIRECT环境被创建")
            self.process_controller.create_env(env_name="DIRECT")
            self.listener_thread_dict['DIRECT'].add_data_queue(self.process_controller.envs['DIRECT']['data_queue'])
            self.listener_thread_dict['DIRECT'].start()  # 开启监听
        except Exception as e:
            print("发生异常 on_direct_environment:", e)

    def on_digital_twin_environment(self):
        try:
            print("Digital Twin环境被创建")
            self.process_controller.create_env(env_name="DT")
            self.embed_pybullet_widget2DT = EmbedPyBullet(win_hwnd=self.process_controller.envs['DT']['win_hwnd'],
                                                           win_name=self.process_controller.envs['DT']['win_name'],
                                                           embed_type="")
            self.frame_DT.layout().addWidget(self.embed_pybullet_widget2DT)
            self.listener_thread_dict['DT'].add_data_queue(self.adsCommunicatorDialog.read_queue)
            self.listener_thread_dict['DT'].start()  # 开启监听
            self.listener_thread_dict['DT_ENV'].add_data_queue(self.process_controller.envs['DT']['data_queue'])
            self.listener_thread_dict['DT_ENV'].start()
            # self.embed_pybullet_widget.resize(self.frame_DT.size())  # 确保嵌入窗口的大小与 frame 相同

        except Exception as e:
            print("发生异常 Digital Twin环境被选中:", e)


    def on_start_all_environments(self):
        print("启动所有环境")

    def on_terminate_env(self):
        try:
            action = self.sender()  # 获取触发信号的动作
            if action.data() in self.process_controller.envs:
                print("Shutting down env:{}".format(action.data()))
                self.process_controller.send_command(env_name=action.data(), command='terminate')
                self.process_controller.stop_env(env_name=action.data())
                p_env_names = []
                for p in processes_info:
                    p_env_names.append(p)
                print(f'Exist processes: {p_env_names}')
                if action.data() == 'GUI':
                    self.embed_pybullet_widget2GUI.close()
                elif action.data() == 'DIRECT':
                    self.embed_pybullet_widget2DIRECT.close()
                else:
                    self.embed_pybullet_widget2DT.close()
            else:
                QMessageBox.critical(self, "Error!", f"该env还未创建: {str(action.data())}")
        except Exception as e:
            print(f"ERROR in on_terminate_env {str(e)}:", e)
        pass
    def on_load_robot(self):
        try:
            action = self.sender()  # 获取触发信号的动作
            if action.data() in self.process_controller.envs:
                print("load robot for {}".format(action.data()))
                # self.pybullet_controller.load_robot(env_name="DT")
                # file_path = "F:\sw\\urdf_files\i7-nofork.SLDASM\\urdf\i7-nofork.SLDASM.urdf"
                file_path = "F:\python\RobotGUI_2.1\\User_Defined_Demos\QT_ADS_DT_demo\\resources\\urdf_files\Ether_CAT\\urdf\Ether_CAT_no_inertia.urdf"
                self.process_controller.send_command(action.data(), 'load_robot', file_path, (0, 0, 0), True, 0)
            else:
                QMessageBox.critical(self, "Error!", f"该env还未创建: {str(action.data())}")
        except Exception as e:
            print(f"ERROR in on_load_robot {str(e)}:", e)
            # if action.data() == "DT":
            #     if 'DT' in self.process_controller.envs:
            #         print("load robot for DT")
            #         # self.pybullet_controller.load_robot(env_name="DT")
            #         file_path = "F:\sw\\urdf_files\i7-nofork.SLDASM\\urdf\i7-nofork.SLDASM.urdf"
            #         self.process_controller.send_command('DT','load_robot',file_path, (0, 0, 0), True, 0)
            #     else:
            #         QMessageBox.critical(self, "Error", f"该env还未创建: {str(action.data())}")
            # elif action.data() == "GUI":
            #     if 'GUI' in self.process_controller.envs:
            #         print("load robot for GUI")
            #         file_path = "F:\sw\\urdf_files\i7-nofork.SLDASM\\urdf\i7-nofork.SLDASM.urdf"
            #         self.process_controller.send_command('GUI', 'load_robot', file_path, (0, 0, 0), True, 0)
            #         # self.pybullet_controller.load_robot(env_name="GUI")
            #     else:
            #         QMessageBox.critical(self, "Error", f"该env还未创建: {str(action.data())}")
            # elif action.data() == "DIRECT":
            #     print("load robot for DIRECT")
            #     file_path = "F:\sw\\urdf_files\i7-nofork.SLDASM\\urdf\i7-nofork.SLDASM.urdf"
            #     self.process_controller.send_command('DIRECT', 'load_robot', file_path, (0, 0, 0), True, 0)
            #     # self.pybullet_controller.load_robot(env_name="DIRECT")
            # else:
            #     print("请选择正确的环境以加载robot")


        pass

    # def on_run_GUI_env(self):
    #     try:
    #         if 'GUI' not in self.process_controller.envs:
    #             raise ValueError("GUI environment has not been created.")
    #
    #
    #         run_flag = self.process_controller.envs['GUI']['Run'].value
    #         if run_flag:
    #             self.process_controller.send_command('GUI','Pause')
    #             self.pushButton_run_GUI.setText('Run GUI')
    #         else:
    #             self.process_controller.send_command('GUI','Run')
    #             self.pushButton_run_GUI.setText('Pause GUI')
    #     except ValueError as e:
    #         QMessageBox.critical(self, "Error", str(e))
    #     except Exception as e:
    #         # 捕获其他未预见的异常
    #         QMessageBox.critical(self, "Error", f"An unexpected error occurred: {str(e)}")
    #         pass
    #
    #     pass

    def on_run_env(self):
        button = self.sender()  # 获取触发信号的按钮
        env_name = button.objectName()  # 获取按钮对象的名称，用作环境名
        try:
            if env_name not in self.process_controller.envs:
                raise ValueError(f"{env_name} environment has not been created.")

            run_flag = self.process_controller.envs[env_name]['Run'].value

            if run_flag:    # 暂停仿真
                self.process_controller.send_command(env_name, 'Pause')
                button.setText(f'Run {env_name}')
            else:   # 启动仿真
                self.process_controller.send_command(env_name, 'Run')

                # self.listener_thread_dict[env_name].start() # 开启监听
                button.setText(f'Pause {env_name}')
        except ValueError as e:
            QMessageBox.critical(self, "Error", str(e))
        except Exception as e:
            QMessageBox.critical(self, "Error", f"An unexpected error occurred: {str(e)}")


    def on_set_robot_data(self):
        button = self.sender()  # 获取触发信号的按钮
        button_name = button.objectName()  # 获取按钮对象的
        try:
            data_str = button.line_edit.text()
            data = [float(x) for x in data_str.split(',')]
            data_name = button.line_edit.objectName()
            if 'GUI' in data_name:
                if 'pos' in data_name:
                    self.process_controller.send_command('GUI','set_pos',data)
                elif 'ori' in data_name:
                    self.process_controller.send_command('GUI', 'set_ori',data)
                elif 'joints_angle' in data_name:
                    self.process_controller.send_command('GUI', 'set_joint_angle', data)
            pass
        except Exception as e:
            print(f"ERROR in set robot data {str(e)}")
            pass
        pass


    def update_GUI_data(self,robot_messages):
        try:
            if not robot_messages.isNone:
                self.GUI_latest_robot_messages = robot_messages
                joint_states = robot_messages.joint_states
                text_joints = ', '.join([f"{joint:.2f}" for joint in joint_states])
                pos, ori = robot_messages.end_effector_pos_and_ori[0], robot_messages.end_effector_pos_and_ori[1]
                text_pos = ', '.join([f"{_:.2f}" for _ in pos])
                text_ori = ', '.join([f"{_:.2f}" for _ in ori])
                if robot_messages.additional_info == 'Pause':
                    text_pos = f"Pause:{text_pos}"
                    text_ori = f"Pause:{text_ori}"
                    text_joints = f"Pause:{text_joints}"
                self.GUI_pos_data.setText(text_pos)
                self.GUI_ori_data.setText(text_ori)
                self.GUI_joints_angle_data.setText(text_joints)

            else:
                self.GUI_pos_data.setText('No data received')
                self.GUI_ori_data.setText('No data received')
                self.GUI_joints_angle_data.setText('No data received')
                pass
        except Exception as e:
            print(f"Error updating GUI data: {str(e)}")
        pass


    def run_GUI_path_list(self):
        try:
            selected_item = self.GUI_path_data_ListWidget.currentItem()
            if selected_item:
                path_name = selected_item.text()
                if 'GUI' in self.process_controller.envs:
                    self.process_controller.send_command('GUI', 'run_joint_angle_list_path', self.GUI_paths_dict[path_name])
                else:
                    print('GUI env is not exist!')
            else:
                print('No path selected!')
        except Exception as e:
            print(f"Error run_GUI_path_list: {str(e)}")


    def reset_GUI_robot_state(self):
        try:
            if 'GUI' in self.process_controller.envs:
                self.process_controller.send_command('GUI', 'reset', )
            else:
                print('GUI env is not exist!')

        except Exception as e:
            print(f"Error run_GUI_path_list: {str(e)}")
        pass

    def send_GUI_path_list(self):
        import pyads
        try:
            AMS_NET_ID = '5.74.55.154.1.1'
            AMS_PORT = 851
            self.plc = pyads.Connection(AMS_NET_ID, AMS_PORT)
            self.plc.open()

            for joint_angles in self.GUI_paths_dict['Unnamed1']:
                for i in range(6):
                    self.plc.write_by_name(f'MAIN.Move_ACS_M{i + 1}_py', True, pyads.PLCTYPE_BOOL)
                for i in range(6):
                    self.plc.write_by_name(f'MAIN.MovePos_ACS_M{i+1}_py', joint_angles[i], pyads.PLCTYPE_LREAL)
                for i in range(6):
                    self.plc.write_by_name(f'MAIN.Move_ACS_M{i + 1}_py', False, pyads.PLCTYPE_BOOL)

        except Exception as e:
            print(f"ERROR in send_GUI_path_list: {str(e)}")


    def tansform_unit(self,joint_states,type='py_send'):
        pybullet_selection = "R" if self.pybullet_buttons["R"].isChecked() else "D"
        robot_arm_selection = "R" if self.robot_arm_buttons["R"].isChecked() else "D"
        if type=='py_send':
            if pybullet_selection == "R" and robot_arm_selection == "D":
                joint_states_n = [math.degrees(radian) for radian in joint_states]
            elif pybullet_selection == "D" and robot_arm_selection == "R":
                joint_states_n = [math.radians(radian) for radian in joint_states]
            else:
                joint_states_n = joint_states
        elif type=='py_received':
            if pybullet_selection == "R" and robot_arm_selection == "D":
                joint_states_n = [math.radians(radian) for radian in joint_states]
            elif pybullet_selection == "D" and robot_arm_selection == "R":
                joint_states_n = [math.degrees(radian) for radian in joint_states]
            else:
                joint_states_n = joint_states
        else:
            joint_states_n = joint_states
        return joint_states_n
        pass


    def send_GUI_state(self):
        try:
            # 判断是否创建ads通讯
            if self.adsCommunicatorDialog.Plc and self.adsCommunicatorDialog.Plc.is_open:
                if self.GUI_latest_robot_messages.joint_states:

                    joint_states = list(self.GUI_latest_robot_messages.joint_states)
                    joint_states = self.tansform_unit(joint_states)
                    python_pos_names = []
                    python_enable_names = []
                    if self.adsCommunicatorDialog.var_dict:
                        for details in self.adsCommunicatorDialog.var_dict.values():
                            python_var_name = details['python_var']
                            if 'movepos_' in python_var_name:
                                python_pos_names.append(python_var_name)
                            elif 'move_' in python_var_name:
                                python_enable_names.append(python_var_name)
                        sorted_pos_names = sorted(python_pos_names,
                                                  key=lambda name: int(re.search(r'm(\d+)', name).group(1)))
                        sorted_enable_names = sorted(python_enable_names,
                                                  key=lambda name: int(re.search(r'm(\d+)', name).group(1)))
                        if len(python_pos_names) == len(joint_states) and len(python_enable_names) == len(joint_states):
                            # 进行python变量更新
                            self.adsCommunicatorDialog.handleVariableSChange(sorted_pos_names, joint_states)
                            # 使能脉冲
                            # self.adsCommunicatorDialog.handleVariableSChange(sorted_enable_names, [False for _ in range(len(python_enable_names))])
                            self.adsCommunicatorDialog.Plc.write_by_name('MAIN.Move_Acs_M_py',False,pyads.PLCTYPE_BOOL)
                            time.sleep(0.01)
                            # self.adsCommunicatorDialog.handleVariableSChange(sorted_enable_names, [True for _ in range(len(python_enable_names))])
                            self.adsCommunicatorDialog.Plc.write_by_name('MAIN.Move_Acs_M_py', True, pyads.PLCTYPE_BOOL)


                        else:
                            print("python位置变量数量与关节角数量不对应")
                    else:
                        print('adsCommunicatorDialog.var_dict is None')
                else:
                    print('robot_messages.joint_states is None')
                pass
            else:
                print('ADS 通讯未创建')

        except Exception as e:
            print(f"ERROR in send_GUI_state: {str(e)}")


    def update_DT_data(self,robot_messages):
        try:
            if not robot_messages.isNone:
                joint_states = robot_messages.joint_states
                text_joints = ', '.join([f"{joint:.2f}" for joint in joint_states])
                pos, ori = robot_messages.end_effector_pos_and_ori[0], robot_messages.end_effector_pos_and_ori[1]
                text_pos = ', '.join([f"{_:.2f}" for _ in pos])
                text_ori = ', '.join([f"{_:.2f}" for _ in ori])
                if robot_messages.additional_info == 'Pause':
                    text_pos = f"Pause:{text_pos}"
                    text_ori = f"Pause:{text_ori}"
                    text_joints = f"Pause:{text_joints}"
                self.DT_pos_data.setText(text_pos)
                self.DT_ori_data.setText(text_ori)
                self.DT_joints_angle_data.setText(text_joints)
            else:
                self.DT_pos_data.setText('No data received')
                self.DT_ori_data.setText('No data received')
                self.DT_joints_angle_data.setText('No data received')
                pass
        except Exception as e:
            print(f"Error updating DT data: {str(e)}")
        pass


    def copy_data(self):
        self.current_button = self.sender()  # 获取触发信号的按钮

        try:
            """复制位置数据到剪贴板"""
            data = self.current_button.line_edit.text()  # 通过按钮访问 QLineEdit
            QApplication.clipboard().setText(data)
            # # 显示对钩图标来表示复制成功
            # icon_size = self.GUI_copy_pos_btn.sizeHint()
            # self.GUI_copy_pos_btn.setIcon(QIcon(QPixmap('resources/images/green.png').scaled(icon_size, transformMode=Qt.SmoothTransformation)))
            # self.GUI_copy_pos_btn.setLayoutDirection(Qt.RightToLeft)
            # self.GUI_copy_pos_btn.setIconSize(icon_size)
            self.current_button.setText('Copy √!')
            # 设置定时器清除图标
            QTimer.singleShot(500, self.clear_icon)  # 2秒后清除图标
            # QMessageBox.information(self, "Copied", "Position data copied to clipboard!")
        except Exception as e:
            print(f"Error copy pos data: {str(e)}")


    def on_create_connection(self):
        button = self.sender()
        if not button.isconnected:
            button.setText("Has been connected! click to disconnect")
            button.isconnected = True
            # 连接信号与槽，以实时更新数据
            # self.listener_thread_dict['GUI'].data_received.connect(self.send_data_to_DT)
            self.listener_thread_dict['DT'].data_received.connect(self.send_data_to_DT) # 将DT listener监听的数据发给DT环境
        else:
            button.setText("Create connection")
            button.isconnected = False
            # 断开信号与槽的连接
            # self.listener_thread_dict['GUI'].data_received.disconnect(self.send_data_to_DT)
            self.listener_thread_dict['DT'].data_received.disconnect(self.send_data_to_DT)
        pass

    def send_data_to_DT(self, robot_messages):
        try:
            if self.connect_to_DT_btn.isconnected:
                # 这里假设DT环境已经在self.listener_thread_dict中创建了相应的队列
                # 将数据发送到DT环境的命令队列
                if 'DT' in self.process_controller.envs and not robot_messages.isNone:
                    robot_messages.joint_states = self.tansform_unit(robot_messages.joint_states,type='py_received')
                    self.process_controller.send_command('DT', 'set_joint_angle', robot_messages.joint_states)

        except Exception as e:
            print(f"ERROR in send_data_to_DT {str(e)}")

    def clear_icon(self):
        try:
            # 清除按钮的图标
            # self.GUI_copy_pos_btn.setIcon(QIcon())
            self.current_button.setText('Copy')
        except Exception as e:
            print(f"Error clear icon: {str(e)}")

    def init_cmd_input(self):
        self.textEdit_cmd_input.setPlainText(">>> ")
        self.textEdit_cmd_input.moveCursor(QtGui.QTextCursor.End)

    def eventFilter(self, source, event):
        if source == self.textEdit_cmd_input:
            if event.type() == QtCore.QEvent.KeyPress:
                if event.key() in (QtCore.Qt.Key_Return, QtCore.Qt.Key_Enter):
                    self.handle_command()
                    return True
                elif event.key() == QtCore.Qt.Key_Backspace:
                    if self.textEdit_cmd_input.textCursor().position() <= 4:
                        return True  # Ignore backspace at prompt
        return super(MainWindow, self).eventFilter(source, event)

    def handle_command(self):
        try:
            text = self.textEdit_cmd_input.toPlainText()
            # 移除最后的 '>>> ' 和任何额外的空白
            command = text.split('>>> ')[-1].strip()
            if command:
                self.textEdit_cmd.append(">>> " + command)
                # 使用 ExecuteCmd 实例执行命令
                result, error = self.executor.execute_command(command)
                self.textEdit_cmd_input.add_current_command_to_history()    # 添加历史列表
                if not error:
                    print(f'Command has been executed:{result}')
                self.textEdit_cmd.append(result)
            self.init_cmd_input()


        except Exception as e:
            print(f"ERROR in handle_command{str(e)}")
            self.init_cmd_input()



    def on_clear_logs(self):
        try:
            self.textEdit_cmd.clear()
        except Exception as e:
            print(f"ERROR in on_clear_logs:{str(e)}")
        pass

    def on_save_logs(self):
        try:
            options = QFileDialog.Options()
            options |= QFileDialog.DontUseNativeDialog
            default_dir = 'F:/python/RobotGUI_2.1/User_Defined_Demos/QT_ADS_DT_demo/logs'  # Default directory set to user's home directory

            if not QDir(default_dir).exists():
                default_dir = QDir.homePath()  # Fallback to home directory if custom path doesn't exist
            fileName, _ = QFileDialog.getSaveFileName(self, "Save Log", default_dir,
                                                      "Text Files (*.txt);;All Files (*)", options=options)
            if fileName:
                # Write text from textEdit_cmd to the chosen file
                with open(fileName, 'w') as file:
                    file.write(self.textEdit_cmd.toPlainText())
                QMessageBox.information(self, "Save Log", "Log saved successfully.")

            pass
        except Exception as e:
            print(f"ERROR in on_save_logs:{str(e)}")

    def receive_path_data(self, path_name, path_data):
        self.GUI_paths_dict[path_name] = path_data
        self.GUI_path_data_ListWidget.addItem(path_name)
        print(f"Received path data for {path_name}")

    def update_variable(self, var_name, value):
        """ 更新ADSCommunicator中的变量 """
        if var_name in self.adsCommunicatorDialog._dynamic_vars or hasattr(self.adsCommunicatorDialog, var_name):
            self.adsCommunicatorDialog.set_variable_value(var_name, value)
        else:
            print(f"Variable {var_name} does not exist. Creating new.")
            self.adsCommunicatorDialog.set_variable_value(var_name, value)  # 创建新变量



    def openDirectEnvDialog(self):
        self.directEnvDialog.show()  # 显示子窗口

    def openADSDialog(self):
        self.ads_thread.adsCommunicatorDialog.show()
        pass

    def cmd_helper(self,help_info):

        pass

    def handleVariableUpdate(self):

        pass




def main():
    # from Robot.Machine import main as create_machine
    # create_machine()
    app = QApplication(sys.argv)
    mainWindow = MainWindow()
    mainWindow.show()
    sys.exit(app.exec_())

def test1():
    app = QApplication(sys.argv)
    mainWindow = MainWindow()
    mainWindow.handle_command()
    pass

if __name__ == '__main__':
    main()
