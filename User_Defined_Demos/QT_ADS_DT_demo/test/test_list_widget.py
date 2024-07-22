from PyQt5.QtWidgets import QMainWindow, QApplication, QListWidget, QMenu, QAction, QVBoxLayout, QWidget
from PyQt5.QtCore import Qt

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.centralWidget = QWidget(self)
        self.setCentralWidget(self.centralWidget)
        self.layout = QVBoxLayout(self.centralWidget)

        self.listWidget = QListWidget(self)
        self.layout.addWidget(self.listWidget)
        self.listWidget.setContextMenuPolicy(Qt.CustomContextMenu)
        self.listWidget.customContextMenuRequested.connect(self.showContextMenu)

        # 添加示例项
        self.listWidget.addItem("GUI环境:")
        self.listWidget.addItem("DIRECT环境:")
        self.listWidget.addItem("Digital Twin:")

    def showContextMenu(self, position):
        item = self.listWidget.itemAt(position)
        menu = QMenu()

        if item is not None:
            if item.text() == "Digital Twin:":
                # 为 Digital Twin 环境添加特定的菜单项
                loadRobotAction = QAction('Load Robot for DT', self)
                shutdownAction = QAction('Shutdown DT', self)
                menu.addAction(loadRobotAction)
                menu.addAction(shutdownAction)
                loadRobotAction.triggered.connect(self.loadRobotForDT)
                shutdownAction.triggered.connect(self.shutdownDT)
            else:
                # 默认菜单项
                loadRobotAction = QAction('Load Robot', self)
                shutdownAction = QAction('Shutdown', self)
                menu.addAction(loadRobotAction)
                menu.addAction(shutdownAction)
                loadRobotAction.triggered.connect(self.loadRobot)
                shutdownAction.triggered.connect(self.shutdownEnvironment)

        # 显示菜单
        menu.exec_(self.listWidget.mapToGlobal(position))

    def loadRobotForDT(self):
        print("Load Robot action triggered for Digital Twin.")

    def shutdownDT(self):
        print("Shutdown Digital Twin environment action triggered.")

    def loadRobot(self):
        print("Load Robot action triggered.")

    def shutdownEnvironment(self):
        print("Shutdown environment action triggered.")

if __name__ == '__main__':
    app = QApplication([])
    window = MainWindow()
    window.show()
    app.exec_()
