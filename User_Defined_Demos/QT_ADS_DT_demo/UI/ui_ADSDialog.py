
from PyQt5.QtCore import Qt
from PyQt5 import QtWidgets, QtCore, QtGui
from PyQt5.QtGui import QFont

class PathDataDialog(QtWidgets.QDialog):
    def __init__(self, path_data, path_name, parent=None):
        super().__init__(parent)
        self.setWindowTitle(f"Data for {path_name}")
        self.resize(900, 600)  # Adjust size as needed

        layout = QtWidgets.QVBoxLayout(self)


        # Create a table widget
        self.tableWidget = QtWidgets.QTableWidget(self)
        self.tableWidget.setRowCount(len(path_data))
        # Add one more column for step indices
        self.tableWidget.setColumnCount(7)  # Now 7 columns including the step column
        headers = ["Step"] + [f"Joint {i + 1}" for i in range(6)]
        self.tableWidget.setHorizontalHeaderLabels(headers)
        self.tableWidget.verticalHeader().setVisible(False)
        self.tableWidget.setEditTriggers(QtWidgets.QTableWidget.NoEditTriggers)
        self.tableWidget.setSelectionBehavior(QtWidgets.QTableWidget.SelectRows)

        # Populate the table
        for i, angles in enumerate(path_data):
            # Insert the step index
            step_item = QtWidgets.QTableWidgetItem(f"step:{str(i)}")
            step_item.setTextAlignment(QtCore.Qt.AlignCenter)
            step_item.setFlags(QtCore.Qt.ItemIsSelectable | QtCore.Qt.ItemIsEnabled)
            self.tableWidget.setItem(i, 0, step_item)

            # Insert angles
            for j, angle in enumerate(angles):
                item = QtWidgets.QTableWidgetItem(f"{angle:.5f}")
                item.setTextAlignment(QtCore.Qt.AlignCenter)
                if j % 2 == 0:
                    item.setBackground(QtGui.QColor(220, 220, 220))  # Light gray
                else:
                    item.setBackground(QtGui.QColor(245, 245, 245))  # Very light gray
                self.tableWidget.setItem(i, j + 1, item)

        self.tableWidget.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.Stretch)
        layout.addWidget(self.tableWidget)

        # Close button
        close_button = QtWidgets.QPushButton("Close", self)
        close_button.clicked.connect(self.close)
        layout.addWidget(close_button)

    def show(self):
        self.tableWidget.resizeColumnsToContents()  # Resize columns to fit content
        super().show()

class UI_ADSDiaglog(QtWidgets.QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("ADS Communication")
        self.resize(600, 400)
        self.setupUi()

    def setupUi(self):
        Hlayout = QtWidgets.QHBoxLayout()

        layout = QtWidgets.QVBoxLayout()
        layout_show_data = QtWidgets.QVBoxLayout()
        Hlayout.addLayout(layout)
        Hlayout.addLayout(layout_show_data)
        self.create_show_vars_work_bench()
        layout_show_data.addWidget(self.show_vars_group_box)


        # Log TextEdit
        self.create_log_group()
        layout.addWidget(self.log_group_box)
        # AMS NET ID and Port input

        self.create_connect_group()
        Hlayout1 = QtWidgets.QHBoxLayout()
        Hlayout1.addWidget(self.connect_group_box)

        # Import XML Button
        self.create_xml_work_bench()
        self.showPathButton.clicked.connect(self.show_selected_path)
        Hlayout1.addWidget(self.xml_group_box)
        layout.addLayout(Hlayout1)


        self.create_var_work_bench()
        layout.addWidget(self.var_group_box)

        self.create_exec_path_work_bench()
        layout.addWidget(self.exec_path_group_box)

        self.setLayout(Hlayout)

    def create_log_group(self):
        self.log_group_box = QtWidgets.QGroupBox("logs")
        Hlayout = QtWidgets.QHBoxLayout(self.log_group_box)
        self.logTextEdit = QtWidgets.QTextEdit()
        self.logTextEdit.setReadOnly(True)
        Vlayout = QtWidgets.QVBoxLayout()
        self.clear_log_btn = QtWidgets.QPushButton('clear')
        self.save_log_btn = QtWidgets.QPushButton('save')
        Vlayout.addWidget(self.clear_log_btn)
        Vlayout.addWidget(self.save_log_btn)
        Hlayout.addWidget(self.logTextEdit)
        Hlayout.addLayout(Vlayout)
        pass


    def create_connect_group(self):
        self.connect_group_box = QtWidgets.QGroupBox("Connection")
        Vlayout = QtWidgets.QVBoxLayout(self.connect_group_box)
        Hlayout1 = QtWidgets.QHBoxLayout()
        Hlayout2 = QtWidgets.QHBoxLayout()
        Hlayout3 = QtWidgets.QHBoxLayout()
        self.amsNetIdLineEdit = QtWidgets.QLineEdit()
        self.amsNetIdLineEdit.setText('5.74.55.154.1.1')
        self.amsNetIdLineLabel = QtWidgets.QLabel('AMS NET ID')
        self.portLineEdit = QtWidgets.QLineEdit()
        self.portLineLabel = QtWidgets.QLabel('Port')
        self.portLineEdit.setText('851')
        Hlayout1.addWidget(self.amsNetIdLineLabel)
        Hlayout1.addWidget(self.amsNetIdLineEdit)
        Hlayout2.addWidget(self.portLineLabel)
        Hlayout2.addWidget(self.portLineEdit)
        self.connectButton = QtWidgets.QPushButton("Connect")
        self.disconnectButton = QtWidgets.QPushButton("Disconnect")
        Hlayout3.addWidget(self.connectButton)
        Hlayout3.addWidget(self.disconnectButton)
        Vlayout.addLayout(Hlayout1)
        Vlayout.addLayout(Hlayout2)
        Vlayout.addLayout(Hlayout3)
        pass

    def on_context_menu(self, position):
        menu = QtWidgets.QMenu()
        delete_action = menu.addAction("Delete")
        action = menu.exec_(self.associationsListWidget.mapToGlobal(position))
        if action == delete_action:
            self.remove_selected_item()

    def create_remove_path(self, position):
        item = self.pathsListWidget.itemAt(position)
        menu = QtWidgets.QMenu()
        if item is not None:
            menu.addAction(self.remove_path_action)
            menu.exec_(self.pathsListWidget.mapToGlobal(position))

    def create_remove_var(self, position):
        item = self.associationsListWidget.itemAt(position)
        menu = QtWidgets.QMenu()
        if item is not None:
            menu.addAction(self.remove_var_action)
            menu.exec_(self.associationsListWidget.mapToGlobal(position))

    def create_xml_work_bench(self):
        self.xml_group_box = QtWidgets.QGroupBox("XML work bench")
        Vlayout = QtWidgets.QVBoxLayout(self.xml_group_box)
        Hlayout = QtWidgets.QHBoxLayout()
        Hlayout1 = QtWidgets.QHBoxLayout()
        # 创建单选按钮
        self.radioD2R = QtWidgets.QRadioButton("(D2R)")
        self.radioR2D = QtWidgets.QRadioButton("(R2D)")
        self.radioNormal = QtWidgets.QRadioButton("(Normal)")
        self.radioNormal.setChecked(True)  # 默认选项为 Normal
        # 创建按钮组
        self.modeButtonGroup = QtWidgets.QButtonGroup(self)
        self.modeButtonGroup.addButton(self.radioD2R, 1)
        self.modeButtonGroup.addButton(self.radioR2D, 2)
        self.modeButtonGroup.addButton(self.radioNormal, 3)
        Hlayout1.addWidget(self.radioD2R)
        Hlayout1.addWidget(self.radioR2D)
        Hlayout1.addWidget(self.radioNormal)
        self.importXmlButton = QtWidgets.QPushButton("Import XML")
        self.pathsListWidget = QtWidgets.QListWidget()
        Vlayout.addLayout(Hlayout1)
        Vlayout.addWidget(self.importXmlButton)
        Vlayout.addLayout(Hlayout)
        Hlayout.addWidget(self.pathsListWidget)
        Vlayout1 = QtWidgets.QVBoxLayout()
        self.showPathButton = QtWidgets.QPushButton("Show Path Data")
        self.sendtoGUIButton = QtWidgets.QPushButton("Send to GUI")
        self.sendtoPLCButton = QtWidgets.QPushButton("Send to PLC")
        self.sendtoPLCButton.setEnabled(False)
        Vlayout1.addWidget(self.showPathButton)
        Vlayout1.addWidget(self.sendtoGUIButton)
        Vlayout1.addWidget(self.sendtoPLCButton)
        Hlayout.addLayout(Vlayout1)

        self.remove_path_action = QtWidgets.QAction('Remove')
        self.pathsListWidget.setContextMenuPolicy(Qt.CustomContextMenu)
        self.pathsListWidget.customContextMenuRequested.connect(self.create_remove_path)
        pass

    def create_show_vars_work_bench(self):
        self.show_vars_group_box = QtWidgets.QGroupBox("Show Vars Values")
        Vlayout = QtWidgets.QVBoxLayout(self.show_vars_group_box)
        # 变量显示列表
        self.var_table = QtWidgets.QTableWidget(self)
        self.var_table.setColumnCount(5)  # 设置四列
        self.var_table.setHorizontalHeaderLabels(["Plc Var Name", "Py Var Name","Value Type", "Value", "Set Value"])
        self.var_table.verticalHeader().setVisible(False)  # 隐藏垂直标题
        self.var_table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.Stretch)
        self.var_table.verticalHeader().setDefaultSectionSize(24)  # 设置默认行高
        self.var_table.setFont(QFont('Arial', 10))
        self.var_table.setAlternatingRowColors(True)  # 设置交替行颜色
        self.var_table.setMinimumWidth(600)

        self.update_auto_btn = QtWidgets.QPushButton('Update automatically')
        self.update_vars_btn = QtWidgets.QPushButton('Update by hand')
        Hlayout = QtWidgets.QHBoxLayout()
        Hlayout.addWidget(self.update_auto_btn)
        Hlayout.addWidget(self.update_vars_btn)
        Vlayout.addWidget(self.var_table)
        Vlayout.addLayout(Hlayout)
        pass

    def create_var_work_bench(self):
        self.var_group_box = QtWidgets.QGroupBox("Var work bench")
        Vlayout = QtWidgets.QVBoxLayout(self.var_group_box)
        Hlayout1 = QtWidgets.QHBoxLayout()
        Hlayout2 = QtWidgets.QHBoxLayout()
        Vlayout.addLayout(Hlayout1)
        Vlayout.addLayout(Hlayout2)
        # Variable association inputs
        self.plcVarLineEdit = QtWidgets.QLineEdit()
        self.plcVarLabel = QtWidgets.QLabel("PLC Variable Name")
        self.plcVarRead_btn = QtWidgets.QPushButton("Read Var")
        self.pythonVarLineEdit = QtWidgets.QLineEdit()
        self.pythonVarLabel = QtWidgets.QLabel("Python Variable Name")
        self.plcVarSet_btn = QtWidgets.QPushButton("Set Var")
        self.associate_Enable_btn = QtWidgets.QPushButton("Enable Associate")

        # 创建单选按钮
        self.radioReadMode = QtWidgets.QRadioButton("Read Mode")
        self.radioWriteMode = QtWidgets.QRadioButton("Write Mode")
        self.radioReadMode.setChecked(True)  # 默认选项为 Normal
        # 创建按钮组
        self.varModeButtonGroup = QtWidgets.QButtonGroup(self)
        self.varModeButtonGroup.addButton(self.radioReadMode, 1)
        self.varModeButtonGroup.addButton(self.radioWriteMode, 2)
        # 创建type按钮
        self.typeButton = QtWidgets.QPushButton("BOOL", self)
        self.typeButton.clicked.connect(self.toggle_button_text)
        HlayoutButton = QtWidgets.QHBoxLayout()
        HlayoutButton.addWidget(self.associate_Enable_btn)
        HlayoutButton.addWidget(self.radioReadMode)
        HlayoutButton.addWidget(self.radioWriteMode)
        HlayoutButton.addWidget(self.typeButton)
        self.associateButton = QtWidgets.QPushButton("Associate Variables")
        self.associateButton.setEnabled(False)
        Hlayout1.addWidget(self.plcVarLabel)
        Hlayout1.addWidget(self.plcVarLineEdit)
        Hlayout1.addWidget(self.plcVarRead_btn)
        Hlayout2.addWidget(self.pythonVarLabel)
        Hlayout2.addWidget(self.pythonVarLineEdit)
        Hlayout2.addWidget(self.plcVarSet_btn)
        Vlayout.addLayout(Hlayout1)
        Vlayout.addLayout(Hlayout2)
        Vlayout.addLayout(HlayoutButton)
        Vlayout.addWidget(self.associateButton)
        Hlayout3 = QtWidgets.QHBoxLayout()
        # List Widget for associations
        self.associationsListWidget = QtWidgets.QListWidget()
        self.remove_var_action = QtWidgets.QAction('Remove', self)
        self.associationsListWidget.setContextMenuPolicy(Qt.CustomContextMenu)
        self.associationsListWidget.customContextMenuRequested.connect(self.create_remove_var)
        Hlayout3.addWidget(self.associationsListWidget)
        Vlayout_jason = QtWidgets.QVBoxLayout()
        self.load_jason_btn = QtWidgets.QPushButton("load from json")
        self.export_jason_btn = QtWidgets.QPushButton("export to jason")
        self.refresh_btn = QtWidgets.QPushButton("refresh list")
        self.clear_btn = QtWidgets.QPushButton("clear list")
        Vlayout_jason.addWidget(self.load_jason_btn)
        Vlayout_jason.addWidget(self.refresh_btn)
        Vlayout_jason.addWidget(self.clear_btn)
        Vlayout_jason.addWidget(self.export_jason_btn)
        Hlayout3.addLayout(Vlayout_jason)
        # Online associate
        self.Online_associate_btn = QtWidgets.QPushButton("Online associate")
        self.hand_associate_btn = QtWidgets.QPushButton("Associate by hand")



        Vlayout.addLayout(Hlayout3)
        Vlayout.addWidget(self.Online_associate_btn)
        Vlayout.addWidget(self.hand_associate_btn)

        pass

    def create_exec_path_work_bench(self):
        self.exec_path_group_box = QtWidgets.QGroupBox("Exec path work bench")
        Vlayout = QtWidgets.QVBoxLayout(self.exec_path_group_box)
        Hlayout1 = QtWidgets.QHBoxLayout()
        Vlayout1 = QtWidgets.QVBoxLayout()
        Hlayout2 = QtWidgets.QHBoxLayout()
        Vlayout.addLayout(Hlayout1)
        Vlayout.addLayout(Hlayout2)
        # Variable association inputs
        self.control_button_Hlayout = QtWidgets.QHBoxLayout()
        self.exec_path_btn = QtWidgets.QPushButton("Exec path")
        self.pause_path_btn = QtWidgets.QPushButton("Pause")
        self.resume_path_btn = QtWidgets.QPushButton("Resume")
        self.stop_path_btn = QtWidgets.QPushButton("Stop")
        self.exec_path_btn.setEnabled(False)
        self.pause_path_btn.setEnabled(False)
        self.resume_path_btn.setEnabled(False)
        self.stop_path_btn.setEnabled(False)
        self.choose_path_btn = QtWidgets.QPushButton("Choose path")
        self.choose_path_btn.setFixedWidth(180)
        # 添加进度条
        self.progress_bar = QtWidgets.QProgressBar()
        self.progress_bar.setMaximum(100)  # 设定进度条的最大值
        self.progress_bar.setValue(0)  # 初始化进度条的当前值

        self.control_button_Hlayout.addWidget(self.exec_path_btn)
        self.control_button_Hlayout.addWidget(self.pause_path_btn)
        self.control_button_Hlayout.addWidget(self.resume_path_btn)
        self.control_button_Hlayout.addWidget(self.stop_path_btn)
        Hlayout3 = QtWidgets.QHBoxLayout()
        Hlayout3.addWidget(self.choose_path_btn)
        Hlayout3.addWidget(self.progress_bar)

        Vlayout.addLayout(Hlayout3)
        Vlayout.addLayout(self.control_button_Hlayout)
        pass

    def toggle_button_text(self):
        # 切换按钮上的文字
        current_text = self.typeButton.text()
        if current_text == "BOOL":
            self.typeButton.setText("LREAL")
        elif current_text == "LREAL":
            self.typeButton.setText("STRING")
        elif current_text == "STRING":
            self.typeButton.setText("INT")
        elif current_text == "INT":
            self.typeButton.setText("STRUCTURE")
        elif current_text == "STRUCTURE":
            self.typeButton.setText("BOOL")

    def show_selected_path(self):
        selected_item = self.pathsListWidget.currentItem()
        if selected_item:
            path_name = selected_item.text()
            path_data = self.paths_dict[path_name]
            # Create and show the dialog
            dialog = PathDataDialog(path_data, path_name, self)
            dialog.show()
