from PyQt5.QtWidgets import QApplication, QLineEdit, QWidget, QVBoxLayout

class MyWidget(QWidget):
    def __init__(self):
        super().__init__()
        self.initUI()

    def initUI(self):
        # 创建 QLineEdit
        self.line_edit = QLineEdit(self)
        # 启用清除按钮，它会在文本框非空时显示
        self.line_edit.setClearButtonEnabled(True)

        # 创建布局并添加控件
        layout = QVBoxLayout(self)
        layout.addWidget(self.line_edit)

        self.setLayout(layout)
        self.setWindowTitle('QLineEdit with Clear Button')
        self.setGeometry(300, 300, 300, 50)

def main():
    app = QApplication([])
    ex = MyWidget()
    ex.show()
    app.exec_()

if __name__ == '__main__':
    main()
