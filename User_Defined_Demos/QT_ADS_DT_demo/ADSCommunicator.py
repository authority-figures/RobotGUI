import sys
import time
import queue
import re
from PyQt5 import QtWidgets
from PyQt5.QtWidgets import QFileDialog,QMessageBox
from PyQt5.QtCore import pyqtSignal,QThread,QTimer,QDir
from PyQt5.QtGui import QColor
from UI.ui_ADSDialog import UI_ADSDiaglog
import xml.etree.ElementTree as ET
import math
import pyads
import json
import copy
from Process_Controller import RobotMessages
class ADSCommunicatorThread(QThread):
    # 用于更新UI或其他操作的信号
    update_log_signal = pyqtSignal(str)
    update_ui_signal = pyqtSignal()

    def __init__(self, adsCommunicatorDialog):
        super().__init__()
        self.adsCommunicatorDialog = adsCommunicatorDialog
        self.update_log_signal.connect(self.adsCommunicatorDialog.update_log_text)
        self.running = False


    def run(self):
        print('Start Online Associating')
        self.update_log_signal.emit("Start Online Associating.")
        self.timer = QTimer()  # 使用定时器来周期性调用
        self.timer.timeout.connect(self.sync_variables)
        self.timer.setInterval(80)  # 设置定时器间隔为1000毫秒

        self.running = True
        self.timer.start()  # 启动定时器
        self.exec_()  # 开始事件循环

    pass

    def sync_variables(self):
        try:
            if self.running and self.adsCommunicatorDialog.Plc and self.adsCommunicatorDialog.Plc.is_open:
                self.adsCommunicatorDialog.sync_struct()

                self.update_ui_signal.emit()  # 发射信号，请求UI更新
                self.msleep(20)
            else:
                self.update_log_signal.emit("Timer ticked but no action taken.")
                self.adsCommunicatorDialog.Online_associate_btn.setText('Online associate')
                self.msleep(10)
                self.stop()

                # print("Timer ticked but no action taken.")

        except Exception as e:
            print(f"ERROR in ADSCommunicatorThread.sync_variables:{str(e)}")

    def stop(self):
        print('Stop Online Associating')
        self.update_log_signal.emit("Stop Online Associating.")
        self.running = False
        self.timer.stop()  # 停止定时器
        self.quit()  # 结束线程的事件循环


structure_def=(
    ('M1',pyads.PLCTYPE_LREAL,1),
    ('M2',pyads.PLCTYPE_LREAL,1),
    ('M3',pyads.PLCTYPE_LREAL,1),
    ('M4',pyads.PLCTYPE_LREAL,1),
    ('M5',pyads.PLCTYPE_LREAL,1),
    ('M6',pyads.PLCTYPE_LREAL,1),
)



class PathSenderThread(QThread):
    progress_updated = pyqtSignal(int)  # 发送进度值的信号
    def __init__(self, data, adsCommunicatorDialog):
        super().__init__()
        self.data = data
        self.adsCommunicatorDialog = adsCommunicatorDialog  # ADSCommunicator 实例
        self.running = False
        self.ispause = False


    def set_data(self, data):
        self.data = data
        pass

    def run(self):
        try:
            self.running = True
            self.adsCommunicatorDialog.logTextEdit.append("exec path is running.")

            # 这里包含将路径数据发送到 PLC 的逻辑
            if self.data is not None:
                total_steps = len(self.data)
                i=0
                for index, path_step in enumerate(self.data):
                    if i%10 < 0:
                        i +=1
                        continue
                    i+=1

                    flag_up_byhand = self.adsCommunicatorDialog.Online_associate_btn.text() == 'Online associate' and self.adsCommunicatorDialog.Plc and self.adsCommunicatorDialog.Plc.is_open

                    while self.ispause:
                        self.msleep(10)  # 如果处于暂停状态，周期性检查是否应该继续

                    if not self.running:
                        # 进度条清零
                        self.progress_updated.emit(0)
                        break


                    self.adsCommunicatorDialog.exec_path_data(path_step)  # 假设这是发送数据的方法
                    # self.adsCommunicatorDialog.exec_path_enable_data(value=False, Command_ifshow=False)  # 使能脉冲
                    # self.adsCommunicatorDialog.Plc.write_by_name('MAIN.Move_Acs_M_py',False,pyads.PLCTYPE_BOOL)
                    if flag_up_byhand:
                        # self.adsCommunicatorDialog.sync_variables()  # 立即同步变量
                        # self.adsCommunicatorDialog.Plc.write_by_name('MAIN.get_string', True, pyads.PLCTYPE_BOOL)
                        self.adsCommunicatorDialog.Plc.write_by_name('MAIN.Move_Acs_M_py', False, pyads.PLCTYPE_BOOL)
                        self.adsCommunicatorDialog.sync_struct()    # 批量同步
                        # self.adsCommunicatorDialog.Plc.write_by_name('MAIN.get_string', False, pyads.PLCTYPE_BOOL)
                        self.msleep(1)
                        # self.adsCommunicatorDialog.exec_path_enable_data(value=True,Command_ifshow=False)
                        self.adsCommunicatorDialog.Plc.write_by_name('MAIN.Move_Acs_M_py',True,pyads.PLCTYPE_BOOL)
                        self.msleep(5)
                    # self.msleep(10)  # 根据需要调整时间间隔
                    else:
                        joints = ','.join(format(num, '.4f') for num in path_step)
                        joints = joints + ',F'
                        self.adsCommunicatorDialog.write_data_queue.put(joints)   # 点位字符串放入执行队列

                    progress_percent = int((index + 1) / total_steps * 100)
                    self.progress_updated.emit(progress_percent)  # 发送进度更新信号
                self.adsCommunicatorDialog.logTextEdit.append("Exec path has been finished.")
            else:
                print('No Path Data Chosen')
        except Exception as e:
            print(f"ERROR in PathSenderThread.run:{str(e)}")



    def pause(self):
        try:
            self.ispause = True
            self.adsCommunicatorDialog.logTextEdit.append("exec path has been paused.")
        except Exception as e:
            print(f"ERROR in PathSenderThread.pause:{str(e)}")

    def resume(self):
        try:
            self.ispause= False
            self.adsCommunicatorDialog.logTextEdit.append("exec path has been resumed.")
        except Exception as e:
            print(f"ERROR in PathSenderThread.resume:{str(e)}")
    def stop(self):
        try:
            self.running = False
            self.resume()  # 如果线程处于暂停状态，将其唤醒以确保可以正常退出
            self.progress_updated.emit(0)  # 清空进度
            self.adsCommunicatorDialog.logTextEdit.append("exec path has been stopped.")
        except Exception as e:
            print(f"ERROR in PathSenderThread.stop:{str(e)}")

class ADSCommunicator(UI_ADSDiaglog):
    pathDataSignal = pyqtSignal(str, object)  # Path name and data
    ReadvariableUpdated = pyqtSignal(dict) # 发送变量名和值的字典
    WritevariableUpdated = pyqtSignal(dict)  # 发送变量名和值的字典
    variableChangeRequest = pyqtSignal(str, object) # 用于接收外界传入的对python_py的修改

    def __init__(self, parent=None):
        super().__init__(parent)
        self.thread = ADSCommunicatorThread(self)
        self.path_thread = PathSenderThread(None, self)
        self.Plc = None
        self.var_dict = {}
        self.paths_dict = {}
        self.Read_struct_dict = {}
        self.Write_struct_dict = {}
        self._dynamic_vars = {}  # 用于存储动态创建的变量值
        self.exec_path = None
        self.read_queue = queue.Queue()
        self.write_data_queue = queue.Queue()
        self.setup_timer()  # 用于定时对变量进行更新显示

        self.base_connect()


    def base_connect(self):
        self.connectButton.clicked.connect(self.create_connection)
        self.importXmlButton.clicked.connect(self.import_xml)
        self.associateButton.clicked.connect(self.add_association)
        self.remove_var_action.triggered.connect(self.remove_selected_item)
        self.remove_path_action.triggered.connect(self.remove_selected_path)
        self.sendtoGUIButton.clicked.connect(self.send_path_to_GUI)
        self.disconnectButton.clicked.connect(self.close_connection)
        self.Online_associate_btn.clicked.connect(self.start_or_stop_thread)  # 定时器
        self.load_jason_btn.clicked.connect(self.import_variables_from_json)
        self.refresh_btn.clicked.connect(self.update_associations_list)
        self.clear_btn.clicked.connect(self.clear_associations_list)
        self.export_jason_btn.clicked.connect(self.export_variables_to_json)
        self.associate_Enable_btn.clicked.connect(self.enable_associate_var)
        self.clear_log_btn.clicked.connect(self.clear_logs)
        self.save_log_btn.clicked.connect(self.save_logs)
        # self.hand_associate_btn.clicked.connect(self.sync_variables)
        # 带字典的读取
        self.hand_associate_btn.clicked.connect(self.sync_struct)

        # 读写变量
        self.plcVarRead_btn.clicked.connect(self.read_var)
        self.plcVarSet_btn.clicked.connect(self.set_var)

        # 控制线程运行
        self.exec_path_btn.clicked.connect(self.path_thread.start)
        self.pause_path_btn.clicked.connect(self.path_thread.pause)
        self.resume_path_btn.clicked.connect(self.path_thread.resume)
        self.stop_path_btn.clicked.connect(self.path_thread.stop)
        self.choose_path_btn.clicked.connect(self.choose_path_data_to_exec)
        self.path_thread.progress_updated.connect(self.progress_bar.setValue)

        # 显示所有的python_var
        self.thread.update_ui_signal.connect(self.update_variable_display)
        self.update_auto_btn.clicked.connect(self.toggleTimer)
        self.update_vars_btn.clicked.connect(self.update_variable_display)

    def update_log_text(self, text):
        self.logTextEdit.append(text)

    def start_or_stop_thread(self):
        if self.thread.isRunning():
            self.thread.stop()
            self.thread.wait()
            self.Online_associate_btn.setText('Online associate')
        else:
            self.thread.start()
            self.Online_associate_btn.setText('Offline associate')

    def setup_timer(self):
        self.display_timer = QTimer(self)
        self.display_timer.timeout.connect(self.update_variable_display)


    def toggleTimer(self):
        if self.display_timer.isActive():
            self.display_timer.stop()
            self.logTextEdit.append("Timer stopped")
            self.update_auto_btn.setText('Update automatically')
            self.update_vars_btn.setEnabled(True)
        else:
            self.display_timer.start(100)  # 每1000毫秒更新一次
            self.logTextEdit.append("Display Timer started")
            self.update_auto_btn.setText('Stopped Update automatically')
            self.update_vars_btn.setEnabled(False)


    def update_variable_display(self):

        try:
            if not self.var_dict:
                self.var_table.setRowCount(1)
                self.var_table.setSpan(0, 0, 1, self.var_table.columnCount())  # 设置跨越整个表格的单元格
                self.var_table.setItem(0, 0, QtWidgets.QTableWidgetItem('请在var work bench中添加plc-python变量对'))  # 在跨越的单元格中显示提示消息
                return
            self.var_table.clearSpans()  # 取消单元格的跨越显示
            self.var_table.setRowCount(len(self.var_dict))  # 根据变量数量设置行数
            # 先对变量进行排序，Read模式的变量在前
            sorted_vars = sorted(self.var_dict.items(), key=lambda item: (item[1]['mode'] == 'Write', item[0]))
            row = 0
            for plc_var, details in sorted_vars:
                value = getattr(self, details['python_var'], 'Undefined')
                var_type = str(details['type'])
                self.var_table.setItem(row, 0, QtWidgets.QTableWidgetItem(plc_var))
                self.var_table.setItem(row, 1, QtWidgets.QTableWidgetItem(details['python_var']))
                self.var_table.setItem(row, 2, QtWidgets.QTableWidgetItem(var_type))
                self.var_table.setItem(row, 3, QtWidgets.QTableWidgetItem(str(value)))
                self.var_table.item(row, 1).setBackground(QColor(0, 215, 120))

                if details['mode'] == 'Read':
                    self.var_table.setItem(row, 4, QtWidgets.QTableWidgetItem('None'))
                    self.var_table.item(row, 0).setBackground(QColor(215, 120, 0))  # 深色背景
                    self.var_table.item(row, 3).setBackground(QColor(200, 200, 250))  # 浅色背景
                else:  # 'Write'
                    self.var_table.setItem(row, 4, QtWidgets.QTableWidgetItem(''))
                    self.var_table.item(row, 0).setBackground(QColor(0, 120, 215))  # 深色背景
                    self.var_table.item(row, 3).setBackground(QColor(200, 200, 250))  # 浅色背景

                if var_type == 'BOOL': self.var_table.item(row, 2).setBackground(QColor(200, 50, 50))  # 浅色背景
                elif var_type == 'LREAL': self.var_table.item(row, 2).setBackground(QColor(50, 50, 200))  # 浅色背景

                if value == 'Undefined':
                    self.var_table.item(row, 3).setBackground(QColor(100, 100, 100))  # 浅色背景
                row += 1
            # self.var_table.setRowCount(max(row,row)-1)  # 根据变量数量修改行数
        except Exception as e:
            print(f"ERROR in update_variable_display:{str(e)}")

    def on_auto_update_vars_btn(self):
        if self.update_auto_btn.text() == 'Update automatically':
            self.update_auto_btn.setText('Stopped Update automatically')
            self.update_vars_btn.setEnabled(True)
        else:
            self.update_auto_btn.setText('Update automatically')
            self.update_vars_btn.setEnabled(False)

        pass

    def create_connection(self):
        ams_net_id = self.amsNetIdLineEdit.text()
        port = self.portLineEdit.text()

        AmsNetID = self.amsNetIdLineEdit.text()
        port = self.portLineEdit.text()
        try:
            # 打开PLC端口
            self.Plc = pyads.Connection(AmsNetID, eval(port))
            self.Plc.open()
            # 连接成功，输出日志
            self.logTextEdit.append('地址{}，端口{}，请输入变量信息'.format(AmsNetID, port).replace('\n', ''))
        except:
            # 连接失败，输出日志
            self.logTextEdit.append('地址{}，端口{}，端口连接失败'.format(AmsNetID, port).replace('\n', '')
                                   + '\nADS服务未开启或地址端口不合法')

    def close_connection(self):
        try:
            if self.Plc:
                self.Plc.close()
                self.logTextEdit.append("Connection closed.")
            else:
                print("没有已创建的连接")
        except Exception as e:
            print(f"ERROR in close_connection:{str(e)}")




    def import_xml(self):
        default_dir = 'F:/python/RobotGUI_2.1/User_Defined_Demos/QT_ADS_DT_demo/resources/data'
        filename, _ = QtWidgets.QFileDialog.getOpenFileName(self, "Open XML file", default_dir, "XML files (*.xml)")
        if filename:
            selected_mode = self.modeButtonGroup.checkedId()
            mode = 'normal'
            if selected_mode == 1:
                mode = 'D2R'
            elif selected_mode == 2:
                mode = 'R2D'
            self.parse_xml(filename, mode=mode)
            self.logTextEdit.append(f"Imported XML file: {filename}")
            # 这里添加解析XML文件的逻辑

    def parse_xml(self, filename, mode='D2R'):
        tree = ET.parse(filename)
        root = tree.getroot()
        path = []  # Initialize a new path list

        path_name = root.get('name', f'Unnamed{len(self.paths_dict) + 1}')  # Default or provided name

        for step in root.findall('Step'):
            if mode == 'D2R':
                # Convert degrees to radians
                joints = [math.radians(float(step.find(f'Joint{i}').text)) for i in range(1, 7)]
            elif mode == 'R2D':
                # Convert radians to degrees
                joints = [math.degrees(float(step.find(f'Joint{i}').text)) for i in range(1, 7)]
            else:
                # Normal mode, no conversion
                joints = [float(step.find(f'Joint{i}').text) for i in range(1, 7)]

            path.append(tuple(joints))

        self.paths_dict[path_name] = path
        self.pathsListWidget.addItem(path_name)  # Add path name to the list widget

    def import_variables_from_json(self):
        try:
            default_dir = 'F:/python/RobotGUI_2.1/User_Defined_Demos/QT_ADS_DT_demo/resources/data'
            filename, _ = QtWidgets.QFileDialog.getOpenFileName(self, "Open JSON File", default_dir, "JSON Files (*.json)")
            if filename:
                with open(filename, 'r') as file:
                    data = json.load(file)
                    self.var_dict.update(data)  # 将JSON文件中的数据合并到var_dict
                    self.update_associations_list()
        except Exception as e:
            print(f"ERROR in import_variables_from_json:{str(e)}")

    def export_variables_to_json(self):
        filename, _ = QtWidgets.QFileDialog.getSaveFileName(self, "Save JSON File", "", "JSON Files (*.json)")
        if filename:
            with open(filename, 'w') as file:
                json.dump(self.var_dict, file, indent=4)  # 使用indent参数美化输出格式
            self.logTextEdit.append(f"Exported variables to {filename}")

    def update_associations_list(self):
        try:
            self.associationsListWidget.clear()  # 清空现有列表
            for plc_var, details in self.var_dict.items():
                entry = f"{plc_var}::{details['python_var']} | Mode={details['mode']} | Type={details['type']}"

                self.associationsListWidget.addItem(entry)
        except Exception as e:
            print(f"ERROR in update_associations_list:{str(e)}")


    def clear_associations_list(self):
        try:
            for details in self.var_dict.values():
                if hasattr(self, details['python_var']):
                    delattr(self, details['python_var'])
            self.var_dict.clear()
            self._dynamic_vars.clear()
            self.update_associations_list()
        except Exception as e:
            print(f"ERROR in clear_associations_list:{str(e)}")

    def enable_associate_var(self):
        if self.associateButton.isEnabled():
            self.associateButton.setEnabled(False)
            self.plcVarLineEdit.clear()
            self.pythonVarLineEdit.clear()
            self.plcVarRead_btn.setEnabled(True)
            self.plcVarSet_btn.setEnabled(True)
            self.associate_Enable_btn.setText('Enable Read and Set')
        else:
            self.associateButton.setEnabled(True)
            self.plcVarLineEdit.clear()
            self.pythonVarLineEdit.clear()
            self.plcVarRead_btn.setEnabled(False)
            self.plcVarSet_btn.setEnabled(False)
            self.associate_Enable_btn.setText('Enable Associate')
        pass


    def read_var(self):
        try:
            plc_var = self.plcVarLineEdit.text()
            if plc_var and self.Plc and self.Plc.is_open:
                selected_type = self.typeButton.text()
                type = self.get_plc_type(str(selected_type))
                if selected_type=='STRUCTURE':
                    value = self.Plc.read_structure_by_name(plc_var,type,1,pyads.ads.size_of_structure(type))
                else:
                    value = self.Plc.read_by_name(plc_var, type)
                self.pythonVarLineEdit.setText(str(value))
                self.logTextEdit.append(f"read {plc_var} = {value}")

            else:
                self.logTextEdit.append(f"plc_var and self.Plc and self.Plc is False")
        except Exception as e:
            print(f"ERROR in read_var:{str(e)}")

        pass

    def set_var(self):
        try:
            plc_var = self.plcVarLineEdit.text()
            py_var = self.pythonVarLineEdit.text()
            if plc_var and py_var and self.Plc and self.Plc.is_open:
                selected_type = self.typeButton.text()
                type = self.get_plc_type(str(selected_type))
                self.Plc.write_by_name(plc_var, py_var, type)
                self.logTextEdit.append(f"set {py_var} to {plc_var}")
        except Exception as e:
            print(f"ERROR in set_var:{str(e)}")
        pass

    def add_association(self):
        plc_var = self.plcVarLineEdit.text()
        python_var = self.pythonVarLineEdit.text()
        if plc_var and python_var:
            selected_mode = self.varModeButtonGroup.checkedId()
            selected_type = self.typeButton.text()

            if selected_mode == 1:
                mode = 'Read'
            else:
                mode = 'Write'
            self.associationsListWidget.addItem(f"{plc_var}::{python_var} | Mode={str(mode)} | Type={selected_type}")
            self.var_dict[str(plc_var)] = {
                'python_var': str(python_var),
                'type': str(selected_type),  # 转换类型字符串为pyads类型
                'mode': str(mode)
            }
            self.plcVarLineEdit.clear()
            self.pythonVarLineEdit.clear()
            self.logTextEdit.append(f"Associated PLC Variable '{plc_var}' with Python Variable '{python_var}',Mode={str(mode)}")

    def remove_selected_item(self):
        try:
            item = self.associationsListWidget.currentItem()
            if item:
                key = item.text()
                plc_name = str(key.split(':')[0])
                if hasattr(self, self.var_dict[plc_name]['python_var']):
                    delattr(self, self.var_dict[plc_name]['python_var'])
                del self.var_dict[plc_name]

                self.associationsListWidget.takeItem(self.associationsListWidget.row(item))
                self.logTextEdit.append(f"Removed association: {key}")
        except Exception as e:
            print(f"ERROR in remove_selected_item:{str(e)}")

    def remove_selected_path(self):
        item = self.pathsListWidget.currentItem()
        if item:
            name = item.text()
            self.pathsListWidget.takeItem(self.pathsListWidget.row(item))
            self.logTextEdit.append(f"Removed Path: {name}")

    def send_path_to_GUI(self):
        selected_item = self.pathsListWidget.currentItem()
        if selected_item:
            path_name = selected_item.text()
            path_data = self.paths_dict[path_name]
            self.pathDataSignal.emit(path_name, path_data,)  # Emit signal with the path data

    # def toggle_timer(self):
    #     try:
    #         # 控制定时器的启动和停止
    #         if self.timer.isActive():
    #             self.timer.stop()
    #             self.Online_associate_btn.setText('Online associate')
    #         else:
    #             self.timer.start(1000)
    #             self.Online_associate_btn.setText('Offline associate')
    #     except Exception as e:
    #         print(f"ERROR in toggle_timer:{str(e)}")

    def set_variable_value(self, var_name, value):
        """ 安全地设置变量值 """
        setattr(self, var_name, value)
        self._dynamic_vars[var_name] = value  # 更新或添加变量到字典

    def get_variable_value(self, var_name):
        """ 安全地获取变量值 """
        return getattr(self, var_name, None)  # 返回变量值，如果未定义则返回None


    def get_plc_type(self, type_str):
        # 根据类型字符串返回对应的pyads类型
        type_mapping = {
            'LREAL': pyads.PLCTYPE_LREAL,
            'BOOL': pyads.PLCTYPE_BOOL,
            'INT': pyads.PLCTYPE_INT,
            'STRING': pyads.PLCTYPE_STRING,
            'STRUCTURE': structure_def,
            # 更多类型映射
        }
        return type_mapping[type_str]

    def sync_struct(self):
        if not self.Plc or not self.Plc.is_open:
            self.logTextEdit.append("PLC connection is not open.")
            # raise TypeError('PLC connection is not open.')
            return

        read_updates = {}
        write_updates = {}
        try:
            Plc_inputString = []
            for plc_var, details in self.var_dict.items():
                if details['mode'] == 'Write' and self.get_variable_value(details['python_var']) is not None:
                    value = self.get_variable_value(details['python_var'])
                    write_updates[details['python_var']] = value  # 收集更新
                    self.set_variable_value(details['python_var'], None)  # 重置以避免重复写入
                    Plc_inputString.append(value)
                if details['mode'] == 'Read':
                    structure_value = self.Plc.read_structure_by_name(plc_var,structure_def,1,pyads.ads.size_of_structure(structure_def))
                    self.set_variable_value(details['python_var'], structure_value)
                    self.read_queue.put(self.zip_robotmessage(structure_value))  # 将读取的数据放入队列中

            try:
                Plc_inputString = ','.join(format(num, '.4f') for num in Plc_inputString)
                Plc_inputString = Plc_inputString + ',F'
                self.Plc.write_by_name('Main.inputString', Plc_inputString, pyads.PLCTYPE_STRING)
            except ValueError:
                # 如果转换失败（例如由于非数字字符串），打印错误消息
                print("列表包含无法转换为浮点数的元素")
            if read_updates:  # 如果有更新则发送一次信号
                self.ReadvariableUpdated.emit(read_updates)
            if write_updates:
                self.WritevariableUpdated.emit(write_updates)
        except Exception as e:
            print(f"Error in sync_struct: {str(e)}")


    def zip_robotmessage(self,Plc_structure):
        joint_values = []

        for value in Plc_structure.values():
            joint_values.append(value)
        robot_message = RobotMessages(joint_states=joint_values,isNone=False)

        return robot_message
        pass

    def zip_data(self):
        for plc_var, details in self.var_dict.items():
            if details['mode'] == "Read":
                self.Read_struct_dict[plc_var] = None
            else:
                self.Write_struct_dict[plc_var] = self.get_variable_value(details['python_var'])
        pass

    def sync_variables(self):
        if not self.Plc or not self.Plc.is_open:
            self.logTextEdit.append("PLC connection is not open.")
            # raise TypeError('PLC connection is not open.')
            return

        read_updates = {}
        write_updates = {}
        try:

            for plc_var, details in self.var_dict.items():
                type = self.get_plc_type(details['type'])
                if details['mode'] == 'Read':
                    if details['type']=='STRUCTURE':
                        value = self.Plc.read_structure_by_name(plc_var, structure_def, 1,
                                                                          pyads.ads.size_of_structure(structure_def))
                        self.set_variable_value(details['python_var'], value)
                    else:
                        value = self.Plc.read_by_name(plc_var, type)

                    read_updates[details['python_var']] = value  # 收集更新
                elif details['mode'] == 'Write' and str(self.get_variable_value(details['python_var'])) != details['last_value']:
                    value = self.get_variable_value(details['python_var'])
                    self.set_variable_value(details['last_value'],str(value))
                    write_updates[details['python_var']] = value  # 收集更新


                    self.Plc.write_by_name(plc_var, value, type)

                    # self.set_variable_value(details['python_var'], None)  # 重置以避免重复写入


            if read_updates:  # 如果有更新则发送一次信号
                self.ReadvariableUpdated.emit(read_updates)
            if write_updates:
                self.WritevariableUpdated.emit(write_updates)
        except Exception as e:
            print(f"Error in syncing variables: {str(e)}")

    def clear_logs(self):
        try:
            self.logTextEdit.clear()
        except Exception as e:
            print(f"ERROR in clear_logs:{str(e)}")
        pass

    def save_logs(self):
        try:
            options = QFileDialog.Options()
            options |= QFileDialog.DontUseNativeDialog
            default_dir = 'F:/python/RobotGUI_2.1/User_Defined_Demos/QT_ADS_DT_demo/logs'
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
        pass

    def handleVariableSChange(self, var_names=None, values=None):
        # 对单个python变量进行修改
        try:
            if var_names and values:
                for var_name, value in zip(var_names, values):
                    setattr(self, var_name, value)
                if self.Online_associate_btn.text() == 'Online associate':  # 未进行在线绑定
                    self.sync_struct()  # 立即同步变量
                else:
                    self.sync_struct()

        except Exception as e:
            print(f"ERROR in handleVariableChange:{str(e)}")
        pass

    def exec_path_data(self, path_step, ifshow=False):
        try:
            if len(path_step) == 6:
                setattr(self, 'movepos_m1_py', path_step[0])
                setattr(self, 'movepos_m2_py', path_step[1])
                setattr(self, 'movepos_m3_py', path_step[2])
                setattr(self, 'movepos_m4_py', path_step[3])
                setattr(self, 'movepos_m5_py', path_step[4])
                setattr(self, 'movepos_m6_py', path_step[5])
            if ifshow:
                self.logTextEdit.append(f"var1={getattr(self, 'movepos_m1_py', None):.2f},"
                                                              f"var2={getattr(self, 'movepos_m2_py', None):.2f},"
                                                              f"var3={getattr(self, 'movepos_m3_py', None):.2f},"
                                                              f"var4={getattr(self, 'movepos_m4_py', None):.2f},"
                                                              f"var5={getattr(self, 'movepos_m5_py', None):.2f},"
                                                              f"var6={getattr(self, 'movepos_m6_py', None):.2f}.")
        except Exception as e:
            print(f"ERROR in exec_path_data:{str(e)}")
            self.logTextEdit.append(f"ERROR in exec_path_data:{str(e)}")
        pass


    def exec_path_enable_data(self, value=False, ifshow=False):
        try:

            setattr(self, 'move_m1_py', value)
            setattr(self, 'move_m2_py', value)
            setattr(self, 'move_m3_py', value)
            setattr(self, 'move_m4_py', value)
            setattr(self, 'move_m5_py', value)
            setattr(self, 'move_m6_py', value)

            if ifshow:
                self.logTextEdit.append(f"var1={getattr(self, 'move_m1_py', None):.2f},"
                                                              f"var2={getattr(self, 'move_m2_py', None):.2f},"
                                                              f"var3={getattr(self, 'move_m3_py', None):.2f},"
                                                              f"var4={getattr(self, 'move_m4_py', None):.2f},"
                                                              f"var5={getattr(self, 'move_m5_py', None):.2f},"
                                                              f"var6={getattr(self, 'move_m6_py', None):.2f}.")
        except Exception as e:
            print(f"ERROR in exec_path_data:{str(e)}")
            self.logTextEdit.append(f"ERROR in exec_path_data:{str(e)}")
        pass


    def choose_path_data_to_exec(self):
        try:
            selected_item = self.pathsListWidget.currentItem()
            if selected_item:
                path_name = selected_item.text()
                path_data = self.paths_dict[path_name]
                self.exec_path = copy.deepcopy(path_data)
                self.path_thread.set_data(self.exec_path)

                self.exec_path_btn.setEnabled(True)
                self.pause_path_btn.setEnabled(True)
                self.resume_path_btn.setEnabled(True)
                self.stop_path_btn.setEnabled(True)
            else:
                self.exec_path = None
                self.path_thread.set_data(self.exec_path)
                self.exec_path_btn.setEnabled(False)
                self.pause_path_btn.setEnabled(False)
                self.resume_path_btn.setEnabled(False)
                self.stop_path_btn.setEnabled(False)
        except Exception as e:
            print(f"ERROR in choose_path_data_to_exec:{str(e)}")
            self.logTextEdit.append(f"ERROR in choose_path_data_to_exec:{str(e)}")
        pass




if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)
    dialog = ADSCommunicator()
    dialog.show()
    sys.exit(app.exec_())