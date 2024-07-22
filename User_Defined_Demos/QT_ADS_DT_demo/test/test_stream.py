import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QTextEdit, QVBoxLayout, QWidget

class Stream:
    def __init__(self, text_widget):
        self.text_widget = text_widget

    def write(self, message):
        self.text_widget.append(message)  # 将文本追加到 QTextEdit 控件

    def flush(self):
        pass  # 在这个例子中，不需要实现任何内容
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.initUI()

    def initUI(self):
        # 创建 QTextEdit 控件
        self.text_edit = QTextEdit()
        self.text_edit.setReadOnly(True)  # 设置为只读，防止用户编辑

        # 设置布局
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        layout = QVBoxLayout()
        layout.addWidget(self.text_edit)
        central_widget.setLayout(layout)

        # 重定向 print 到 QTextEdit
        sys.stdout = Stream(self.text_edit)

        # 模拟 print 输出
        print("Hello, World!")
        print("All print statements will be redirected here.")

        self.setWindowTitle("Print Redirection Example")
        self.resize(400, 300)
        self.show()
if __name__ == '__main__':
    app = QApplication(sys.argv)
    ex = MainWindow()
    sys.exit(app.exec_())
