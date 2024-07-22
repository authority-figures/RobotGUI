from PyQt5.QtWidgets import QApplication, QLineEdit, QWidget, QVBoxLayout
from PyQt5.QtCore import QEvent, Qt

class LineEdit(QLineEdit):
    def __init__(self, parent=None):
        super(LineEdit, self).__init__(parent)
        self.default_style = "background-color: white; color: black;"
        self.hover_style = "background-color: #A0A0A0; color: white;"  # 悬停时的样式
        self.clicked_style = "background-color: #FFD700; color: black;"  # 点击时的样式
        self.setStyleSheet(self.default_style)
        self.installEventFilter(self)

    def eventFilter(self, obj, event):
        if obj == self:
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

class MyWidget(QWidget):
    def __init__(self):
        super().__init__()
        self.initUI()

    def initUI(self):
        layout = QVBoxLayout(self)
        line_edit = LineEdit()
        layout.addWidget(line_edit)
        self.setLayout(layout)
        self.setWindowTitle('LineEdit Styles Example')
        self.show()

if __name__ == '__main__':
    app = QApplication([])
    ex = MyWidget()
    app.exec_()
