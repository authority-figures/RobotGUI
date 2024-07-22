from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QListWidget, QListWidgetItem, QLabel, QPushButton
from PyQt5.QtGui import QIcon, QPixmap
from PyQt5.QtCore import QSize

class MainWindow(QWidget):
    def __init__(self):
        super().__init__()

        self.initUI()

    def initUI(self):
        layout = QVBoxLayout(self)

        # 创建按钮和菜单
        self.button = QPushButton('创建pybullet环境', self)
        layout.addWidget(self.button)

        # 创建状态列表
        self.listWidget = QListWidget(self)
        self.updateEnvironmentStatus("GUI环境", "待启动", "grey")
        self.updateEnvironmentStatus("DIRECT环境", "待启动", "grey")
        self.updateEnvironmentStatus("Digital Twin环境", "待启动", "grey")
        layout.addWidget(self.listWidget)

        self.setLayout(layout)

    def updateEnvironmentStatus(self, env_name, status, color):
        item = QListWidgetItem(env_name + " - " + status)
        self.listWidget.addItem(item)
        # 设置标志灯颜色
        color_map = {"green": "resources/images/green.png", "red": "resources/images/red.png", "grey": "resources/images/yellow.png"}
        icon = QIcon(color_map[color])
        item.setIcon(icon)
        item.setSizeHint(QSize(100, 40))  # 调整大小以更好地显示状态

if __name__ == '__main__':
    app = QApplication([])
    ex = MainWindow()
    ex.show()
    app.exec_()
