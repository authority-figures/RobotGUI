from PyQt5 import QtWidgets, QtCore,QtGui

class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, parent=None):
        super(MainWindow, self).__init__(parent)
        self.centralwidget = QtWidgets.QWidget(self)
        self.setCentralWidget(self.centralwidget)
        self.layout = QtWidgets.QVBoxLayout(self.centralwidget)

        self.textEdit_cmd_output = QtWidgets.QTextEdit(self.centralwidget)
        self.textEdit_cmd_output.setReadOnly(True)
        self.layout.addWidget(self.textEdit_cmd_output)

        self.textEdit_cmd_input = QtWidgets.QTextEdit(self.centralwidget)
        self.textEdit_cmd_input.setObjectName("textEdit_cmd_input")
        self.layout.addWidget(self.textEdit_cmd_input)
        self.textEdit_cmd_input.installEventFilter(self)

        self.init_cmd_input()

    def init_cmd_input(self):
        self.textEdit_cmd_input.setPlainText(">>> ")
        self.textEdit_cmd_input.moveCursor(QtGui.QTextCursor.End)

    def eventFilter(self, source, event):
        if (source == self.textEdit_cmd_input and event.type() == QtCore.QEvent.KeyPress):
            if event.key() == QtCore.Qt.Key_Return and self.textEdit_cmd_input.hasFocus():
                self.handle_command()
                return True
        return super(MainWindow, self).eventFilter(source, event)

    def handle_command(self):
        text = self.textEdit_cmd_input.toPlainText()
        # 移除最后的 '>>> ' 和任何额外的空白
        command = text.split('>>> ')[-1].strip()
        if command:
            self.textEdit_cmd_output.append(">>> " + command)
        self.init_cmd_input()

if __name__ == '__main__':
    app = QtWidgets.QApplication([])
    window = MainWindow()
    window.show()
    app.exec_()
