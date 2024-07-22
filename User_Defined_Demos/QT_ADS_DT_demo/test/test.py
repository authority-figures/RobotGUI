import sys
from PyQt5.QtWidgets import QMainWindow, QApplication, QPushButton, QMenu, QAction, QLabel, QVBoxLayout, QWidget

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        self.initUI()

    def initUI(self):
        self.setGeometry(300, 300, 600, 400)
        self.setWindowTitle('PyBullet Environment Launcher')

        # 主按钮
        self.launchButton = QPushButton('创建pybullet环境', self)
        self.launchButton.setGeometry(100, 100, 200, 50)

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

        # 连接信号与槽
        self.actionGUI.triggered.connect(lambda: self.launchEnvironment('GUI'))
        self.actionDirect.triggered.connect(lambda: self.launchEnvironment('DIRECT'))
        self.actionDigitalTwin.triggered.connect(lambda: self.launchEnvironment('Digital Twin'))
        self.actionAll.triggered.connect(self.launchAll)

        self.launchButton.setMenu(self.menu)

        # 状态标签
        self.statusLabel = QLabel('', self)
        self.statusLabel.setGeometry(320, 100, 250, 150)
        self.statusLabel.setStyleSheet("QLabel { background-color : white; }")

    def launchEnvironment(self, env_type):
        # 模拟环境启动
        self.statusLabel.setText(f'{env_type}环境正在启动...')
        QApplication.processEvents()
        # 假设环境启动需要一些时间
        import time
        time.sleep(2)  # 模拟延时
        self.statusLabel.setText(f'{env_type}环境已启动！')

    def launchAll(self):
        for action in [self.actionGUI, self.actionDirect, self.actionDigitalTwin]:
            self.launchEnvironment(action.text())

if __name__ == '__main__':
    app = QApplication(sys.argv)
    ex = MainWindow()
    ex.show()
    sys.exit(app.exec_())
