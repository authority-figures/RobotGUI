import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QAction, QToolBar, QStatusBar, QVBoxLayout, QWidget, \
    QGridLayout, QTextEdit
from PyQt5.QtCore import Qt, QTimer, QProcess
from PyQt5.QtGui import QWindow, QKeyEvent
from multiprocessing import Process, Queue
import win32con
import win32gui
import time
from PyQt5.QtWidgets import QLineEdit


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.resize(1200, 800)
        self.setWindowTitle('RobotGUI')
        self.initUI()

        self.win_hwnd = None  # 存储找到的窗口句柄，假设我们只关心第一个窗口
        self.checkWindowTimer = QTimer(self)
        self.checkWindowTimer.timeout.connect(self.checkAndEmbedWindow)
        self.checkWindowTimer.start(100)

        self.process = QProcess()
        self.process.start("python", ["gui_win_1_sim.py", "start"])

    def initUI(self):
        centralWidget = QWidget()
        self.setCentralWidget(centralWidget)
        gridLayout = QGridLayout(centralWidget)  # 使用网格布局

        # 创建四个区域的容器，每个容器都是QWidget
        self.subWindows = [QWidget(centralWidget) for _ in range(4)]
        for i in range(2):
            for j in range(2):
                gridLayout.addWidget(self.subWindows[i * 2 + j], i, j)
                self.subWindows[i * 2 + j].setStyleSheet("background-color: #f0f0f0; border: 1px solid black;")

        self.toolbar = QToolBar("Main Toolbar")
        self.addToolBar(self.toolbar)
        action_list = ['新建', '编辑', '载入', '搜索', '设置', '帮助']
        for action_name in action_list:
            action = QAction(action_name, self)
            self.toolbar.addAction(action)
            action.triggered.connect(self.on_action_triggered)

        self.statusbar = QStatusBar()
        self.setStatusBar(self.statusbar)
        self.statusbar.showMessage('Ready')

    def checkAndEmbedWindow(self):
        try:
            def enumWindows(hwnd, extra):
                if win32gui.IsWindow(hwnd) and win32gui.IsWindowVisible(hwnd) and win32gui.IsWindowEnabled(hwnd):
                    title = win32gui.GetWindowText(hwnd)
                    if title == 'Bullet Physics ExampleBrowser using OpenGL3+ [btgl] Release build':  # 根据实际窗口标题调整
                        self.win_hwnd = hwnd
                        self.embedWindow(0)  # 嵌入到第一个QWidget
                        self.checkWindowTimer.stop()

            win32gui.EnumWindows(enumWindows, None)
        except Exception as e:
            self.statusbar.showMessage(f'Error: {str(e)}')

    def embedWindow(self, index):
        try:
            widget = QWidget.createWindowContainer(QWindow.fromWinId(self.win_hwnd))
            widget.setWindowFlags(Qt.WindowStaysOnTopHint | Qt.FramelessWindowHint)
            if self.win_hwnd:
                win32gui.SetParent(self.win_hwnd, int(self.subWindows[index].winId()))
                win32gui.MoveWindow(self.win_hwnd, 0, 0, self.subWindows[index].width(),
                                    self.subWindows[index].height(), True)
        except Exception as e:
            self.statusbar.showMessage(f'Embedding Failed: {str(e)}')

    def adjustEmbeddedWindow(self, index):
        # 根据QWidget的大小调整嵌入窗口的大小
        if self.win_hwnd:
            win32gui.MoveWindow(self.win_hwnd, 0, 0, self.subWindows[index].width(), self.subWindows[index].height(), True)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        # 主窗口大小变化时调整嵌入窗口的大小
        if self.win_hwnd:
            self.adjustEmbeddedWindow(0)  # 假设嵌入窗口在第一个位置

    def on_action_triggered(self):
        sender = self.sender()
        self.statusbar.showMessage(f'{sender.text()} action triggered')

if __name__ == "__main__":
    app = QApplication(sys.argv)
    mainWin = MainWindow()
    mainWin.show()
    sys.exit(app.exec_())




