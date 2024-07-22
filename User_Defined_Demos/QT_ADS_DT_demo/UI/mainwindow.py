# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'mainwindow.ui'
#
# Created by: PyQt5 UI code generator 5.15.9
#
# WARNING: Any manual changes made to this file will be lost when pyuic5 is
# run again.  Do not edit this file unless you know what you are doing.

import sys
sys.path.append('UI/')
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtWidgets import QMainWindow, QCompleter, QPushButton, QMenu, QAction, QLabel, QVBoxLayout, QWidget,QStatusBar, QFrame,QLineEdit
from PyQt5.QtCore import Qt, QEvent, QStringListModel
from PyQt5.QtGui import QTextCursor
from User_Defined_Demos.QT_ADS_DT_demo.ExecuteCmd import Completer
from ui_ADSDialog import PathDataDialog

colors = {
    "light_gray": "#f0f0f0",
    "dark_gray": "#A0A0A0",
    "blue": "#3333ff",
    "green": "#33cc33",
    "red": "#ff3333",
    "black": "#333333",
    "white": "#ffffff",
    "yellow": "#ffff33",
    "purple": "#9933ff",
    "orange": "#ff9900",
    "pink": "#ff99cc",
    "cyan": "#00ffff",
    "brown": "#996633",
    "gold": "#ffd700",
    "silver": "#c0c0c0",
    "navy": "#000080",
    "indigo": "#4b0082",
    "magenta": "#ff00ff",
    "teal": "#008080",
    "olive": "#808000",
    "light_blue": "#ADD8E6",
    "light_green": "#90EE90",
    "light_yellow": "#FFFFE0",
    "light_purple": "#D8BFD8",
    "light_orange": "#FFDAB9",
    "light_pink": "#FFB6C1",
    "light_cyan": "#E0FFFF",
    "light_brown": "#D2B48C",
    "light_gold": "#FAFAD2",
    "light_silver": "#C0C0C0",
    "light_navy": "#000080",
    "light_indigo": "#4B0082",
    "light_magenta": "#FF00FF",
    "light_teal": "#008080",
    "light_olive": "#808000"
}


class LineEdit(QLineEdit):
    def __init__(self, parent=None,type='output'):
        super(LineEdit, self).__init__(parent)
        self.type = type
        self.output_style = f"""
                                QLineEdit {{
                                font: bold 12px;  /* 设置字体大小 */
                                color: {colors['white']};  /* 设置文字颜色 */
                                border: 2px solid {colors['black']};  /* 设置边框 */
                                border-radius: 5px;  /* 设置边框圆角 */
                                padding: 0px;  /* 设置内边距 */
                                background-color: {colors['silver']};  /* 设置背景色 */
                                }}
                            """
        self.input_style = f"""
                                QLineEdit {{
                                font: 12px;  /* 设置字体大小 */
                                color: {colors['black']};  /* 设置文字颜色 */
                                border: 2px solid {colors['blue']};  /* 设置边框 */
                                border-radius: 5px;  /* 设置边框圆角 */
                                padding: 0px;  /* 设置内边距 */
                                background-color: {colors['white']};  /* 设置背景色 */
                                }}
                            """
        if self.type=='output':
            self.default_style = self.output_style
        elif self.type=='input':
            self.default_style = self.input_style
        else: self.default_style = "background-color: white; color: black;"
        self.hover_style = f"""border: 2px solid {colors['cyan']};  /* 设置边框 */ border-radius: 5px;  /* 设置边框圆角 */"""  # 悬停时的样式
        self.clicked_style = f"""
                                QLineEdit {{
                                font: 12px;  /* 设置字体大小 */
                                color: {colors['black']};  /* 设置文字颜色 */
                                border: 2px solid {colors['blue']};  /* 设置边框 */
                                border-radius: 5px;  /* 设置边框圆角 */
                                padding: 0px;  /* 设置内边距 */
                                background-color: {colors['light_blue']};  /* 设置背景色 */
                                }}
                            """


        self.setStyleSheet(self.default_style)
        self.installEventFilter(self)
        self.clicked = False

    def eventFilter(self, obj, event):
        if obj == self and self.type=='input':  # 只有input才响应鼠标状态
            if event.type() == QEvent.MouseButtonPress:
                self.setStyleSheet(self.clicked_style)
                self.setFocus()  # 确保获取焦点以进入编辑状态
            elif event.type() == QEvent.HoverEnter and not self.hasFocus():
                self.setStyleSheet(self.hover_style)
            elif event.type() == QEvent.HoverLeave and not self.hasFocus():
                self.setStyleSheet(self.default_style)
        return super(LineEdit, self).eventFilter(obj, event)

    def focusInEvent(self, event):
        super().focusInEvent(event)
        self.setStyleSheet(self.clicked_style)  # 获取焦点时设置为点击样式

    def focusOutEvent(self, event):
        super().focusOutEvent(event)
        self.setStyleSheet(self.default_style)  # 失去焦点时恢复默认样式


class AutoCompleteTextEdit(QtWidgets.QTextEdit):
    def __init__(self, parent=None):
        super(AutoCompleteTextEdit, self).__init__(parent)
        self.completer = None  # 初始化时没有自动补全逻辑
        self.history = []  # 命令历史列表
        self.history_index = -1  # 当前历史索引

    def set_completer(self, completer):
        self.completer = completer

    # def keyPressEvent(self, event):
    #     if event.key() == Qt.Key_Tab and self.completer:
    #         cursor = self.textCursor()
    #         cursor.select(QTextCursor.WordUnderCursor)
    #         current_text = cursor.selectedText()
    #         full_text = self.toPlainText()[:cursor.position()]
    #         last_word = full_text.split()[-1] if full_text.split() else ""
    #         suggestions = self.completer.complete(last_word)
    #         if suggestions:
    #             completion = suggestions[0][len(current_text):]
    #             self.insertPlainText(completion + '')
    #         return
    #     super(AutoCompleteTextEdit, self).keyPressEvent(event)

    def keyPressEvent(self, event):
        if event.key() == Qt.Key_Tab and self.completer:
            self.handle_autocomplete(event)
        elif event.key() == Qt.Key_Up:
            self.handle_history_back()
        else:
            super(AutoCompleteTextEdit, self).keyPressEvent(event)



    def handle_autocomplete(self, event):
        if event.key() == Qt.Key_Tab and self.completer:
            cursor = self.textCursor()
            cursor.select(QTextCursor.WordUnderCursor)
            current_word = cursor.selectedText()
            cursor_position = cursor.position() - len(current_word)
            full_text = self.toPlainText()
            last_word = full_text.split()[-1] if full_text.split() else ""  # 去掉>>>后的内容
            prefix_text = last_word[:cursor_position].rsplit('.', 1)[0] if '.' in last_word[:cursor_position] else ""

            suggestions = self.completer.complete(last_word)
            if suggestions:
                # 选择第一个匹配的建议进行补全
                suggested_completion = suggestions[0]
                if '.' in suggested_completion:
                    suggested_completion = suggested_completion.split('.')[-1]
                # 删除当前的单词并替换为补全的单词
                cursor.removeSelectedText()
                cursor.insertText(suggested_completion)
                self.setTextCursor(cursor)

    def handle_history_back(self):
        if self.history:
            if self.history_index == -1:  # 如果当前没有遍历历史
                self.history_index = len(self.history) - 1
            else:
                self.history_index = (self.history_index - 1) % len(self.history)
            command = self.history[self.history_index]
            self.setText(command)
            self.moveCursor(QTextCursor.End)

    def append_to_history(self, command):
        self.history.append(command)
        self.history_index = -1  # 重置历史索引

    def add_current_command_to_history(self):
        current_text = self.toPlainText().strip()
        if current_text and (not self.history or current_text != self.history[-1]):
            self.append_to_history(current_text)






class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(986, 629)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setContextMenuPolicy(QtCore.Qt.DefaultContextMenu)
        self.centralwidget.setAutoFillBackground(False)
        self.centralwidget.setObjectName("centralwidget")
        # 主窗口第一层布局
        self.horizontalLayout = QtWidgets.QHBoxLayout(self.centralwidget)
        self.horizontalLayout.setSpacing(20)
        self.horizontalLayout.setObjectName("horizontalLayout")
        # 主窗口第二层布局
        # 将主窗口分为上下两部分
        self.main_VLayout = QtWidgets.QVBoxLayout(self.centralwidget)
        self.main_VLayout.setSpacing(20)
        self.main_VLayout.setObjectName("main_VLayout")

        self.create_cmd_workbench()

        self.main_HLayout = QtWidgets.QHBoxLayout(self.centralwidget)
        self.main_HLayout.setSpacing(20)
        self.main_HLayout.setObjectName("main_HLayout")

        self.HLayout_GUI = QtWidgets.QHBoxLayout()

        self.verticalLayout_GUI = QtWidgets.QVBoxLayout()
        self.verticalLayout_GUI.setSpacing(20)
        self.verticalLayout_GUI.setObjectName("verticalLayout")
        self.create_GUI_workbench()
        self.GUI_showPathData_btn.clicked.connect(self.show_selected_path)

        self.HLayout_GUI.addLayout(self.verticalLayout_GUI)
        self.HLayout_GUI.addWidget(self.GUI_workbench_group_box)
        # 创建 QFrame 作为 PyBullet 窗口的容器
        # Pybullet GUI窗口所在布局
        self.frame_GUI = QFrame()
        self.frame_GUI.setLineWidth(1)
        self.frame_GUI.setMidLineWidth(1)
        self.frame_GUI.setFrameShape(QFrame.StyledPanel)
        self.frame_GUI.setFrameShadow(QFrame.Raised)
        self.frame_GUI.setStyleSheet("QFrame { background-color: gray; border: 3px solid black; }")
        # self.frame_DT.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        self.frame_GUI_layout = QtWidgets.QVBoxLayout()  # 为 frame 设置布局
        self.frame_GUI_layout.setContentsMargins(0, 0, 0, 0)  # 设置边距为零
        self.frame_GUI.setMinimumHeight(400)
        self.frame_GUI.MaxWidth = 500
        self.frame_GUI.setMaximumWidth(self.frame_GUI.MaxWidth)


        self.frame_GUI.setLayout(self.frame_GUI_layout)  # 将布局设置到 frame 上

        self.create_GUI_robot_info()
        self.verticalLayout_GUI.addWidget(self.frame_GUI)
        self.verticalLayout_GUI.addWidget(self.GUI_robot_info_group_box)


        self.verticalLayout_GUI.setStretch(0,15)
        self.verticalLayout_GUI.setStretch(1, 5)

        # self.GUI_Widget = QtWidgets.QWidget(self.centralwidget)
        # self.GUI_Widget.setObjectName("GUI_Widget")
        # self.GUI_Widget.setStyleSheet("background-color: #f0f0f0; border: 1px solid black;")
        # self.verticalLayout_GUI.addWidget(self.GUI_Widget)

        self.verticalLayout = QtWidgets.QVBoxLayout()
        self.verticalLayout.setSpacing(20)
        self.verticalLayout.setObjectName("verticalLayout")
        self.verticalLayout_5 = QtWidgets.QVBoxLayout()
        self.verticalLayout_5.setObjectName("verticalLayout_5")
        self.pushButton_create_env = QtWidgets.QPushButton(self.centralwidget)
        self.pushButton_create_env.setObjectName("pushButton_create_env")
        self.verticalLayout_5.addWidget(self.pushButton_create_env)

        self.HLayout_3button = QtWidgets.QHBoxLayout()
        self.HLayout_3button.setSpacing(20)
        self.HLayout_3button.setObjectName("HLayout_3button")

        self.listWidget_env_status = QtWidgets.QListWidget(self.centralwidget)
        self.listWidget_env_status.setEnabled(True)
        self.listWidget_env_status.setResizeMode(QtWidgets.QListView.Fixed)
        self.listWidget_env_status.setViewMode(QtWidgets.QListView.ListMode)
        self.listWidget_env_status.setModelColumn(0)
        self.listWidget_env_status.setUniformItemSizes(False)
        self.listWidget_env_status.setBatchSize(100)
        self.listWidget_env_status.setObjectName("listWidget_env_status")
        # 为listwidget创建连接
        self.listWidget_env_status.setContextMenuPolicy(Qt.CustomContextMenu)
        self.listWidget_env_status.customContextMenuRequested.connect(self.showContextMenu)

        item = QtWidgets.QListWidgetItem()
        item.setTextAlignment(QtCore.Qt.AlignLeading|QtCore.Qt.AlignVCenter)
        icon = QtGui.QIcon()
        icon.addPixmap(QtGui.QPixmap("resources/images/yellow.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        item.setIcon(icon)
        self.listWidget_env_status.addItem(item)
        item = QtWidgets.QListWidgetItem()
        self.listWidget_env_status.addItem(item)
        item = QtWidgets.QListWidgetItem()
        self.listWidget_env_status.addItem(item)
        self.verticalLayout_5.addWidget(self.listWidget_env_status)
        self.verticalLayout.addLayout(self.verticalLayout_5)


        # 添加三个Run按钮
        self.pushButton_run_GUI = QPushButton(self.centralwidget)
        self.pushButton_run_GUI.setObjectName("GUI")
        self.pushButton_run_DIRECT = QPushButton(self.centralwidget)
        self.pushButton_run_DIRECT.setObjectName("DIRECT")
        self.pushButton_run_DT = QPushButton(self.centralwidget)
        self.pushButton_run_DT.setObjectName("DT")


        self.HLayout_3button.addWidget(self.pushButton_run_GUI)
        self.HLayout_3button.addWidget(self.pushButton_run_DIRECT)
        self.HLayout_3button.addWidget(self.pushButton_run_DT)
        self.verticalLayout.addLayout(self.HLayout_3button)

        self.create_DIRECT_env()
        self.create_control_workbench()
        self.verticalLayout.addWidget(self.DIRECT_workbench_group_box)
        # 添加弹簧
        spacerItem = QtWidgets.QSpacerItem(20, 40, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding)
        self.verticalLayout.addItem(spacerItem)
        self.verticalLayout.addWidget(self.control_workbench_group_box)


        self.verticalLayout.setStretch(0, 2)    # verticalLayout_5 :create env|env list|
        self.verticalLayout.setStretch(1, 4)    # 3button
        self.verticalLayout.setStretch(2, 2)    # DIREACT group
        self.verticalLayout.setStretch(3, 12)  # spring
        # 将verticalLayout添加到main_HLayout中
        self.main_HLayout.addLayout(self.verticalLayout)


        self.main_HLayout.addLayout(self.HLayout_GUI)
        self.main_HLayout.setStretch(0,2)
        self.main_HLayout.setStretch(1, 8)
        # self.main_VLayout.setStretch(2, 2)
        # 左边第一垂直layout
        self.main_VLayout.addLayout(self.main_HLayout)
        # self.main_VLayout.addLayout(self.verticalLayout)
        self.main_VLayout.addStretch()
        self.main_VLayout.addLayout(self.verticalLayout_cmd)
        self.main_VLayout.setStretch(0, 12)
        self.main_VLayout.setStretch(1, 2)
        self.main_VLayout.setStretch(2, 6)
        # 向第一层中添加布局
        # self.horizontalLayout.addLayout(self.verticalLayout)
        self.horizontalLayout.addLayout(self.main_VLayout)

        # spacerItem3 = QtWidgets.QSpacerItem(600, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        # self.horizontalLayout.addItem(spacerItem3)

        self.verticalLayout_3 = QtWidgets.QVBoxLayout()
        self.verticalLayout_3.setSizeConstraint(QtWidgets.QLayout.SetDefaultConstraint)
        self.verticalLayout_3.setContentsMargins(-1, 0, -1, -1)
        self.verticalLayout_3.setSpacing(20)
        self.verticalLayout_3.setObjectName("verticalLayout_3")
        self.verticalLayout_2 = QtWidgets.QVBoxLayout()
        self.verticalLayout_2.setObjectName("verticalLayout_2")
        self.label_display_DT = QtWidgets.QLabel(self.centralwidget)
        self.label_display_DT.setAlignment(QtCore.Qt.AlignCenter)
        self.label_display_DT.setObjectName("label_display_DT")
        self.verticalLayout_2.addWidget(self.label_display_DT)
        self.verticalLayout_3.addLayout(self.verticalLayout_2)
        self.verticalLayout_embed_DT = QtWidgets.QVBoxLayout()
        self.verticalLayout_embed_DT.setObjectName("verticalLayout_embed_DT")
        # 创建 QFrame 作为 PyBullet 窗口的容器
        self.frame_DT = QFrame()
        self.frame_DT.setLineWidth(3)
        self.frame_DT.setMidLineWidth(7)
        self.frame_DT.setFrameShape(QFrame.StyledPanel)
        self.frame_DT.setFrameShadow(QFrame.Raised)
        self.frame_DT.setStyleSheet("QFrame { background-color: gray; border: 3px solid black; }")
        # self.frame_DT.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        self.frame_DT_layout = QtWidgets.QVBoxLayout()  # 为 frame 设置布局
        self.frame_DT_layout.setContentsMargins(0, 0, 0, 0)  # 设置边距为零
        self.frame_DT.setLayout(self.frame_DT_layout)   # 将布局设置到 frame 上
        self.frame_DT.setMinimumWidth(500)  # 设置DT窗口最小宽度
        self.frame_DT.setMinimumHeight(500)  # 设置DT窗口最小高度
        self.frame_DT.MaxWidth = 800
        self.frame_DT.setMaximumWidth(self.frame_DT.MaxWidth)
        self.connect_to_DT_btn = QPushButton('Create connection')
        self.connect_to_DT_btn.isconnected = False
        self.create_DT_robot_info()
        self.verticalLayout_embed_DT.addWidget(self.frame_DT)
        self.verticalLayout_embed_DT.addWidget(self.connect_to_DT_btn)
        self.verticalLayout_embed_DT.addWidget(self.DT_robot_info_group_box)

        self.verticalLayout_3.addLayout(self.verticalLayout_embed_DT)
        self.verticalLayout_3.setStretch(0, 1)
        self.verticalLayout_3.setStretch(1, 19)
        self.horizontalLayout.addLayout(self.verticalLayout_3)
        self.horizontalLayout.setStretch(0, 12)
        self.horizontalLayout.setStretch(1, 8)
        # self.horizontalLayout.setStretch(2, 10)
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 986, 22))
        self.menubar.setObjectName("menubar")
        self.menuw = QtWidgets.QMenu(self.menubar)
        self.menuw.setObjectName("menuw")
        MainWindow.setMenuBar(self.menubar)
        self.menubar.addAction(self.menuw.menuAction())

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)



        self.create_env_menu()

    def create_unit_group(self, title, option1, option2, default=1):
        # 创建一个组框
        group_box = QtWidgets.QGroupBox(title)
        layout = QtWidgets.QHBoxLayout()

        # 创建单选按钮
        rad_btn = QtWidgets.QRadioButton(option1)
        deg_btn = QtWidgets.QRadioButton(option2)

        # 默认选择第一个按钮
        if default==1:
            rad_btn.setChecked(True)
        else:
            deg_btn.setChecked(True)

        # 将按钮添加到布局中
        layout.addWidget(rad_btn)
        layout.addWidget(deg_btn)

        # 设置组框的布局
        group_box.setLayout(layout)

        return group_box, {option1: rad_btn, option2: deg_btn}

    def create_GUI_workbench(self):
        self.GUI_workbench_group_box = QtWidgets.QGroupBox("GUI workbench")
        self.GUI_workbench_layout = QVBoxLayout(self.GUI_workbench_group_box)
        self.GUI_base_layout = QVBoxLayout()

        Vlayout = QVBoxLayout()
        # 添加第一组按钮：Pybullet 单位选择
        self.pybullet_group, self.pybullet_buttons = self.create_unit_group("Py Units:", "R", "D")
        Vlayout.addWidget(self.pybullet_group)

        # 添加第二组按钮：机械臂电机单位选择
        self.robot_arm_group, self.robot_arm_buttons = self.create_unit_group("Plc Units:", "R", "D",default=2)
        Vlayout.addWidget(self.robot_arm_group)

        self.GUI_showPathData_btn = QtWidgets.QPushButton("Show Path Data")
        Vlayout.addWidget(self.GUI_showPathData_btn)
        self.GUI_runPath_btn = QtWidgets.QPushButton("Run Path")
        Vlayout.addWidget(self.GUI_runPath_btn)
        self.GUI_reset_btn = QtWidgets.QPushButton("Reset Robot State")
        Vlayout.addWidget(self.GUI_reset_btn)
        self.GUI_send_path_btn = QtWidgets.QPushButton("Send Path")
        Vlayout.addWidget(self.GUI_send_path_btn)
        self.GUI_send_state_btn = QtWidgets.QPushButton("Send State")
        Vlayout.addWidget(self.GUI_send_state_btn)

        Hlayout = QtWidgets.QHBoxLayout()
        self.GUI_path_data_ListWidget = QtWidgets.QListWidget()
        Hlayout.addWidget(self.GUI_path_data_ListWidget)
        Hlayout.addLayout(Vlayout)



        self.GUI_test_btn1 = QPushButton("test1")
        self.GUI_test_btn2 = QPushButton("test2")
        self.GUI_base_layout.addLayout(Hlayout)
        self.GUI_base_layout.addWidget(self.GUI_test_btn1)
        self.GUI_base_layout.addWidget(self.GUI_test_btn2)
        self.GUI_workbench_layout.addLayout(self.GUI_base_layout)
        pass

    def create_DIRECT_env(self):
        self.DIRECT_workbench_group_box = QtWidgets.QGroupBox("DIRECT workbench")
        self.DIRECT_workbench_layout = QVBoxLayout(self.DIRECT_workbench_group_box)
        self.DIRECT_base_layout = QVBoxLayout()
        self.DIRECT_workbench_layout.addLayout(self.DIRECT_base_layout)
        self.DIRECT_base_layout1 = QtWidgets.QHBoxLayout()
        self.DIRECT_base_layout.addLayout(self.DIRECT_base_layout1)
        self.DIRECT_test_btn1 = QPushButton("test1")
        self.DIRECT_test_btn2 = QPushButton("test2")
        self.DIRECT_base_layout1.addWidget(self.DIRECT_test_btn1)
        self.DIRECT_base_layout1.addWidget(self.DIRECT_test_btn2)
        pass
    
    def create_control_workbench(self):
        self.control_workbench_group_box = QtWidgets.QGroupBox("control workbench")
        self.control_workbench_layout = QVBoxLayout(self.control_workbench_group_box)
        self.control_base_layout = QVBoxLayout()
        self.control_workbench_layout.addLayout(self.control_base_layout)
        self.control_base_layout1 = QtWidgets.QHBoxLayout()
        self.control_base_layout.addLayout(self.control_base_layout1)
        self.control_test_btn1 = QPushButton("Send Command")
        self.control_test_btn2 = QPushButton("ADS Communicator")
        self.control_base_layout1.addWidget(self.control_test_btn1)
        self.control_base_layout1.addWidget(self.control_test_btn2)
        pass

    def create_cmd_workbench(self):
        self.cmd_group_box = QtWidgets.QGroupBox("cmd window")
        self.verticalLayout_cmd = QtWidgets.QVBoxLayout()
        self.verticalLayout_cmd.setSpacing(20)
        self.verticalLayout_cmd.setObjectName("verticalLayout_cmd")
        self.HLayout_cmd = QtWidgets.QHBoxLayout(self.cmd_group_box)
        self.verticalLayout_cmd.addLayout(self.HLayout_cmd)
        self.VLayout_left_cmd = QtWidgets.QVBoxLayout()
        self.VLayout_right_cmd = QtWidgets.QVBoxLayout()
        self.HLayout_cmd.addLayout(self.VLayout_left_cmd)
        self.HLayout_cmd.addLayout(self.VLayout_right_cmd)
        self.clear_cmd_log_btn = QPushButton('clear logs')
        self.save_cmd_log_btn = QPushButton('save logs')
        self.textEdit_cmd = QtWidgets.QTextEdit(self.centralwidget)
        self.textEdit_cmd.setObjectName("textEdit_cmd")
        self.textEdit_cmd_input = AutoCompleteTextEdit(self)
        self.textEdit_cmd_input.setObjectName("textEdit_cmd_input")
        self.textEdit_cmd_input.setMaximumHeight(50)

        self.VLayout_right_cmd.addWidget((self.clear_cmd_log_btn))
        self.VLayout_right_cmd.addWidget((self.save_cmd_log_btn))
        self.VLayout_left_cmd.addWidget(self.textEdit_cmd)
        self.VLayout_left_cmd.addWidget(self.textEdit_cmd_input)
        self.verticalLayout_cmd.addWidget(self.cmd_group_box)
        pass
    
    def create_DT_robot_info(self):
        # 创建显示数据的组
        self.DT_robot_info_group_box = QtWidgets.QGroupBox("DT Robot Arm Data")
        self.DT_robot_info_group_box.setMaximumWidth(self.frame_DT.MaxWidth)
        self.DT_data_layout = QVBoxLayout(self.DT_robot_info_group_box)

        self.DT_pos_and_ori_layout = QtWidgets.QHBoxLayout()  # label copy set 水平布局
        self.DT_pos_and_ori_data_layout = QtWidgets.QHBoxLayout()  # data set_data 水平布局
        self.DT_pos_and_ori_label = QLabel("Position and Orientation:")
        self.DT_pos_data = LineEdit(type='output')
        self.DT_pos_data.setReadOnly(True)
        self.DT_pos_data.setFocusPolicy(Qt.NoFocus)
        self.DT_ori_data = LineEdit(type='output')
        self.DT_ori_data.setReadOnly(True)
        self.DT_ori_data.setFocusPolicy(Qt.NoFocus)

        self.DT_joints_angle_layout = QtWidgets.QHBoxLayout()
        self.DT_joints_angle_data_layout = QtWidgets.QHBoxLayout()
        self.DT_joints_angle_label = QLabel("Joints Angles:")
        self.DT_joints_angle_data = LineEdit(type='output')
        self.DT_joints_angle_data.setReadOnly(True)
        self.DT_joints_angle_data.setFocusPolicy(Qt.NoFocus)

        self.DT_pos_and_ori_layout.addWidget(self.DT_pos_and_ori_label)
        self.DT_pos_and_ori_data_layout.addWidget(self.DT_pos_data)
        self.DT_pos_and_ori_data_layout.addWidget(self.DT_ori_data)
        self.DT_joints_angle_layout.addWidget(self.DT_joints_angle_label)
        self.DT_joints_angle_data_layout.addWidget(self.DT_joints_angle_data)
        self.DT_data_layout.addLayout(self.DT_pos_and_ori_layout)
        self.DT_data_layout.addLayout(self.DT_pos_and_ori_data_layout)
        self.DT_data_layout.addLayout(self.DT_joints_angle_layout)
        self.DT_data_layout.addLayout(self.DT_joints_angle_data_layout)
        spacerItem = QtWidgets.QSpacerItem(20, 40, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding)
        self.DT_data_layout.addItem(spacerItem)

        pass

    def create_GUI_robot_info(self):
        # 创建显示数据的组
        self.GUI_robot_info_group_box = QtWidgets.QGroupBox("GUI Robot Arm Data")
        self.GUI_robot_info_group_box.setMaximumWidth(self.frame_GUI.MaxWidth)
        self.GUI_data_layout = QVBoxLayout(self.GUI_robot_info_group_box)

        # 创建数据显示
        self.GUI_pos_layout = QtWidgets.QHBoxLayout()   # label copy set 水平布局
        self.GUI_pos_data_layout = QtWidgets.QHBoxLayout()  # data set_data 水平布局
        self.GUI_pos_label = QLabel("Position:")
        self.GUI_pos_data = LineEdit(type='output')
        self.GUI_set_pos_data = LineEdit(type='input')
        self.GUI_set_pos_data.setObjectName('GUI_pos_data')
        self.GUI_pos_data.setReadOnly(True)
        self.GUI_pos_data.setFocusPolicy(Qt.NoFocus)
        self.GUI_copy_pos_btn = QPushButton("Copy")
        self.GUI_copy_pos_btn.line_edit = self.GUI_pos_data
        self.GUI_set_pos_btn = QPushButton("Set")
        self.GUI_set_pos_btn.line_edit = self.GUI_set_pos_data
        self.GUI_pos_layout.addWidget(self.GUI_pos_label)
        self.GUI_pos_layout.addWidget(self.GUI_copy_pos_btn)
        self.GUI_pos_layout.addWidget(self.GUI_set_pos_btn)
        self.GUI_pos_data_layout.addWidget(self.GUI_pos_data)
        self.GUI_pos_data_layout.addWidget(self.GUI_set_pos_data)

        self.GUI_ori_layout = QtWidgets.QHBoxLayout()
        self.GUI_ori_data_layout = QtWidgets.QHBoxLayout()
        self.GUI_ori_label = QLabel("Orientation:")
        self.GUI_ori_data = LineEdit(type='output')
        self.GUI_set_ori_data = LineEdit(type='input')
        self.GUI_set_ori_data.setObjectName('GUI_ori_data')
        self.GUI_ori_data.setReadOnly(True)
        self.GUI_ori_data.setFocusPolicy(Qt.NoFocus)
        self.GUI_copy_ori_btn = QPushButton("Copy")
        self.GUI_copy_ori_btn.line_edit = self.GUI_ori_data
        self.GUI_set_ori_btn = QPushButton("Set")
        self.GUI_set_ori_btn.line_edit = self.GUI_set_ori_data
        self.GUI_ori_layout.addWidget(self.GUI_ori_label)
        self.GUI_ori_layout.addWidget(self.GUI_copy_ori_btn)
        self.GUI_ori_layout.addWidget(self.GUI_set_ori_btn)
        self.GUI_ori_data_layout.addWidget(self.GUI_ori_data)
        self.GUI_ori_data_layout.addWidget(self.GUI_set_ori_data)

        self.GUI_joints_angle_layout = QtWidgets.QHBoxLayout()
        self.GUI_joints_angle_data_layout = QtWidgets.QHBoxLayout()
        self.GUI_joints_angle_label = QLabel("Joints Angles:")
        self.GUI_joints_angle_data = LineEdit(type='output')
        self.GUI_set_joints_angle_data = LineEdit(type='input')
        self.GUI_set_joints_angle_data.setObjectName('GUI_joints_angle_data')
        self.GUI_joints_angle_data.setReadOnly(True)
        self.GUI_joints_angle_data.setFocusPolicy(Qt.NoFocus)
        self.GUI_copy_joints_angle_btn = QPushButton("Copy")
        self.GUI_copy_joints_angle_btn.line_edit = self.GUI_joints_angle_data
        self.GUI_set_joints_angle_btn = QPushButton("Set")
        self.GUI_set_joints_angle_btn.line_edit = self.GUI_set_joints_angle_data
        self.GUI_joints_angle_layout.addWidget(self.GUI_joints_angle_label)
        self.GUI_joints_angle_layout.addWidget(self.GUI_copy_joints_angle_btn)
        self.GUI_joints_angle_layout.addWidget(self.GUI_set_joints_angle_btn)
        self.GUI_joints_angle_data_layout.addWidget(self.GUI_joints_angle_data)
        self.GUI_joints_angle_data_layout.addWidget(self.GUI_set_joints_angle_data)

        # 添加组件到布局
        self.GUI_data_layout.addLayout(self.GUI_pos_layout)
        self.GUI_data_layout.addLayout(self.GUI_pos_data_layout)
        self.GUI_data_layout.addLayout(self.GUI_ori_layout)
        self.GUI_data_layout.addLayout(self.GUI_ori_data_layout)
        self.GUI_data_layout.addLayout(self.GUI_joints_angle_layout)
        self.GUI_data_layout.addLayout(self.GUI_joints_angle_data_layout)
        pass

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.pushButton_create_env.setText(_translate("MainWindow", "创建pybullet环境"))
        __sortingEnabled = self.listWidget_env_status.isSortingEnabled()
        self.listWidget_env_status.setSortingEnabled(False)
        item = self.listWidget_env_status.item(0)
        item.setText(_translate("MainWindow", "GUI环境:"))
        item = self.listWidget_env_status.item(1)
        item.setText(_translate("MainWindow", "DIRECT环境:"))
        item = self.listWidget_env_status.item(2)
        item.setText(_translate("MainWindow", "Digital Twin:"))
        self.listWidget_env_status.setSortingEnabled(__sortingEnabled)
        # self.pushButton_2.setText(_translate("MainWindow", "PushButton"))
        # self.pushButton_4.setText(_translate("MainWindow", "PushButton"))
        # self.pushButton_5.setText(_translate("MainWindow", "PushButton"))
        # self.pushButton_3.setText(_translate("MainWindow", "PushButton"))
        self.pushButton_run_GUI.setText(_translate("MainWindow", "Run GUI"))
        self.pushButton_run_DIRECT.setText(_translate("MainWindow", "Run DIRECT"))
        self.pushButton_run_DT.setText(_translate("MainWindow", "Run DT"))
        self.label_display_DT.setText(_translate("MainWindow", "Digital Twin display"))
        self.menuw.setTitle(_translate("MainWindow", "文件"))

        self.textEdit_cmd_input.setStyleSheet(
            f"""
                background-color: {colors['black']};
                font: bold 20px;
                color: green; 
                font-family: Consolas;
                border: 2px solid {colors['dark_gray']};  /* 设置边框 */
                """
        )

        # 设置 QLineEdit 的样式
        self.output_style = f"""
                    QLineEdit {{
                    font: bold 12px;  /* 设置字体大小 */
                    color: {colors['white']};  /* 设置文字颜色 */
                    border: 2px solid {colors['black']};  /* 设置边框 */
                    border-radius: 5px;  /* 设置边框圆角 */
                    padding: 0px;  /* 设置内边距 */
                    background-color: {colors['silver']};  /* 设置背景色 */
                    }}
                    """
        self.input_style = f"""
                            QLineEdit {{
                            font: 12px;  /* 设置字体大小 */
                            color: {colors['black']};  /* 设置文字颜色 */
                            border: 2px solid {colors['blue']};  /* 设置边框 */
                            border-radius: 5px;  /* 设置边框圆角 */
                            padding: 0px;  /* 设置内边距 */
                            background-color: {colors['white']};  /* 设置背景色 */
                            }}
                            """
        self.GUI_pos_data.setStyleSheet(self.output_style)
        self.GUI_pos_data.setMinimumWidth(180)
        self.GUI_ori_data.setStyleSheet(self.output_style)
        self.GUI_ori_data.setMinimumWidth(180)
        self.GUI_joints_angle_data.setStyleSheet(self.output_style)
        self.GUI_joints_angle_data.setMinimumWidth(250)
        self.GUI_set_pos_data.setStyleSheet(self.input_style)
        self.GUI_set_pos_data.setClearButtonEnabled(True)
        self.GUI_set_pos_data.setMinimumWidth(160)
        self.GUI_set_ori_data.setStyleSheet(self.input_style)
        self.GUI_set_ori_data.setClearButtonEnabled(True)
        self.GUI_set_ori_data.setMinimumWidth(160)
        self.GUI_set_joints_angle_data.setStyleSheet(self.input_style)
        self.GUI_set_joints_angle_data.setClearButtonEnabled(True)
        self.GUI_set_joints_angle_data.setMinimumWidth(160)
        self.DT_pos_data.setStyleSheet(self.output_style)
        self.DT_ori_data.setStyleSheet(self.output_style)
        self.DT_joints_angle_data.setStyleSheet(self.output_style)


    def create_env_menu(self):
        # 创建下拉菜单
        self.menu = QMenu(self)
        self.actionGUI = QAction('GUI环境', self)
        self.actionDirect = QAction('DIRECT环境', self)
        self.actionDigitalTwin = QAction('Digital Twin环境', self)
        self.actionAll = QAction('启动全部环境', self)

        # 添加动作到菜单
        self.menu.addAction(self.actionGUI)
        self.menu.addAction(self.actionDirect)
        self.menu.addAction(self.actionDigitalTwin)
        self.menu.addAction(self.actionAll)

        # self.menu.setFixedWidth(self.pushButton_create_env.width())

        self.pushButton_create_env.setMenu(self.menu)

        # self.menu_GUI = QMenu()
        # self.menu_DIRECT = QMenu()
        # self.menu_DT = QMenu()
        self.actionLoadRobot_DT = QAction('Load Robot for DT', self)
        self.actionLoadRobot_DT.setData("DT")  # 将环境名称作为数据存储
        self.actionShutDown_DT = QAction('Shutdown DT', self)
        self.actionShutDown_DT.setData("DT")  # 将环境名称作为数据存储
        self.actionLoadRobot_GUI = QAction('Load Robot for GUI', self)
        self.actionLoadRobot_GUI.setData("GUI")  # 将环境名称作为数据存储
        self.actionShutDown_GUI = QAction('Shutdown GUI', self)
        self.actionShutDown_GUI.setData("GUI")  # 将环境名称作为数据存储
        self.actionLoadRobot_DIRECT = QAction('Load Robot for DIRECT', self)
        self.actionLoadRobot_DIRECT.setData("DIRECT")  # 将环境名称作为数据存储
        self.actionShutDown_DIRECT = QAction('Shutdown DIRECT', self)
        self.actionShutDown_DIRECT.setData("DIRECT")  # 将环境名称作为数据存储

        pass

    def showContextMenu(self, position):
        item = self.listWidget_env_status.itemAt(position)
        menu = QMenu()
        if item is not None:
            if item.text() == "Digital Twin:":
                # 为 Digital Twin 环境添加特定的菜单项

                menu.addAction(self.actionLoadRobot_DT)
                menu.addAction(self.actionShutDown_DT)
                menu.exec_(self.listWidget_env_status.mapToGlobal(position))
            elif item.text() == "GUI环境:":
                menu.addAction(self.actionLoadRobot_GUI)
                menu.addAction(self.actionShutDown_GUI)
                menu.exec_(self.listWidget_env_status.mapToGlobal(position))
            elif item.text() == "DIRECT环境:":
                menu.addAction(self.actionLoadRobot_DIRECT)
                menu.addAction(self.actionShutDown_DIRECT)
                menu.exec_(self.listWidget_env_status.mapToGlobal(position))

                # # 默认菜单项
                # loadRobotAction = QAction('Load Robot', self)
                # shutdownAction = QAction('Shutdown', self)
                # menu.addAction(loadRobotAction)
                # menu.addAction(shutdownAction)
                # loadRobotAction.triggered.connect(self.loadRobot)
                # shutdownAction.triggered.connect(self.shutdownEnvironment)
                pass

        # 显示菜单


    def show_selected_path(self):
        selected_item = self.GUI_path_data_ListWidget.currentItem()
        if selected_item:
            path_name = selected_item.text()
            path_data = self.GUI_paths_dict[path_name]
            # Create and show the dialog
            dialog = PathDataDialog(path_data, path_name,self)
            dialog.show()
