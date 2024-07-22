from PyQt5.QtWidgets import QMainWindow, QApplication, QAction, QMenu, QListWidget, QVBoxLayout, QWidget
from PyQt5.QtCore import Qt


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Environment Control")
        self.setGeometry(100, 100, 600, 400)

        self.listWidget = QListWidget(self)
        self.listWidget.setGeometry(50, 50, 500, 300)

        self.menu = QMenu(self)

        # 创建动作并关联到通用槽函数
        environments = ["GUI环境", "DIRECT环境", "Digital Twin环境"]
        for env in environments:
            action = QAction(f'Load Robot for {env}', self)
            action.setData(env)  # 存储环境名称
            action.triggered.connect(self.on_load_robot)
            self.menu.addAction(action)

        self.listWidget.setContextMenuPolicy(Qt.CustomContextMenu)
        self.listWidget.customContextMenuRequested.connect(self.showContextMenu)

    def showContextMenu(self, position):
        self.menu.exec_(self.listWidget.mapToGlobal(position))

    def on_load_robot(self):
        action = self.sender()  # 获取触发信号的动作
        env_name = action.data()  # 从动作中获取环境名称
        print(f"Loading robot into {env_name}")  # 根据环境名称执行加载操作

        # 这里可以添加将机器人加载到指定环境的代码
        # 例如: self.pybullet_controller.load_robot(env_name=env_name)


if __name__ == '__main__':
    app = QApplication([])
    window = MainWindow()
    window.show()
    app.exec_()
