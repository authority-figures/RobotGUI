import sys
import time

import win32gui
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QLabel
from PyQt5.QtCore import QTimer
from PyQt5 import QtCore
from PyQt5.QtGui import QWindow,QKeyEvent
import win32con

class EmbedPyBullet(QWidget):
    def __init__(self,win_hwnd=None,win_name="Bullet Physics ExampleBrowser using OpenGL3+ [btgl] Release build",embed_type = "Container"):
        super().__init__()
        self.win_name = win_name
        self.embed_type = embed_type
        self.setWindowTitle('PyBullet Embedding Example')
        layout = QVBoxLayout(self)

        # 添加一个标签显示信息
        self.label = QLabel("PyBullet should be embedded below")
        layout.addWidget(self.label,alignment=QtCore.Qt.AlignCenter)

        # 设置定时器检查PyBullet窗口
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.embed_pybullet)
        self.timer.start(10)  # 每1000毫秒检查一次
        self.bullet_hwnd = win_hwnd

    def embed_pybullet(self):
        if not self.bullet_hwnd:
            self.bullet_hwnd = win32gui.FindWindow(None, 'Bullet Physics ExampleBrowser using OpenGL3+ [btgl] Release build')
        if self.bullet_hwnd:
            if self.embed_type == "Container":
                # 将外部窗口嵌入到 PyQt 应用中
                bullet_window = QWindow.fromWinId(self.bullet_hwnd)
                container = QWidget.createWindowContainer(bullet_window, self)
                container.setMinimumSize(800, 600)  # 设置嵌入窗口的最小大小
                self.layout().addWidget(container)
                self.label.deleteLater()  # 删除标签
                self.label = None  # 确保引用被清除
                self.timer.stop()  # 停止定时器，因为已经找到并嵌入了窗口

            else:
                win32gui.SetParent(self.bullet_hwnd, int(self.winId()))
                win32gui.MoveWindow(self.bullet_hwnd, 0, 0, self.width(), self.height(), True)
                win32gui.ShowWindow(self.bullet_hwnd, win32con.SW_MAXIMIZE)
                self.timer.stop()  # 停止定时器，因为已经找到并嵌入了窗口

    def resizeEvent(self, event):

        if self.bullet_hwnd and self.embed_type != "Container":
            # 重置嵌入窗口的大小以匹配当前 widget 的大小

            win32gui.MoveWindow(self.bullet_hwnd, 0, 0, self.width(), self.height(), True)
        super(EmbedPyBullet, self).resizeEvent(event)

    def keyPressEvent(self, event):
        if self.bullet_window_container:
            # 创建一个新的键盘事件，转发到嵌入的窗口
            new_event = QKeyEvent(event.type(), event.key(), event.modifiers())
            QApplication.sendEvent(self.bullet_window_container, new_event)
        super(EmbedPyBullet, self).keyPressEvent(event)

    def close_embedded_window(self):
        if self.bullet_hwnd:
            win32gui.SetParent(self.bullet_hwnd, None)  # 将父窗口设置为空
            self.bullet_hwnd = None
        if self.timer.isActive():
            self.timer.stop()
        if self.label:
            self.label.deleteLater()
        for child in self.findChildren(QWidget):
            child.deleteLater()  # 删除所有子部件


if __name__ == '__main__':

    app = QApplication(sys.argv)
    window = EmbedPyBullet(embed_type="")
    window.show()
    app.exec_()
