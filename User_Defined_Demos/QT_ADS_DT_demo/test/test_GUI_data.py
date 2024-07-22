from PyQt5.QtWidgets import (QMainWindow, QLineEdit, QVBoxLayout, QHBoxLayout,
                             QWidget, QGroupBox, QLabel, QPushButton, QApplication, QMessageBox)
from PyQt5.QtCore import Qt

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Robot Arm Data Display")
        self.setGeometry(100, 100, 400, 300)  # x, y, width, height

        # 主部件和布局
        self.central_widget = QWidget(self)
        self.setCentralWidget(self.central_widget)
        self.layout = QVBoxLayout(self.central_widget)

        # 创建显示数据的组
        self.data_group_box = QGroupBox("Robot Arm Data")
        self.data_layout = QVBoxLayout(self.data_group_box)

        # 创建位置数据显示
        self.pos_layout = QHBoxLayout()
        self.pos_label = QLabel("Position:")
        self.pos_data = QLineEdit()
        self.pos_data.setReadOnly(True)
        self.pos_data.setFocusPolicy(Qt.NoFocus)

        self.copy_pos_btn = QPushButton("Copy")
        self.copy_pos_btn.clicked.connect(self.copy_pos_data)

        # 添加组件到位置数据布局
        self.pos_layout.addWidget(self.pos_label)
        self.pos_layout.addWidget(self.pos_data)
        self.pos_layout.addWidget(self.copy_pos_btn)

        # 创建方向数据显示
        self.ori_label = QLabel("Orientation:")
        self.ori_data = QLineEdit()
        self.ori_data.setReadOnly(True)
        self.ori_data.setFocusPolicy(Qt.NoFocus)

        # 添加组件到布局
        self.data_layout.addLayout(self.pos_layout)
        self.data_layout.addWidget(self.ori_label)
        self.data_layout.addWidget(self.ori_data)

        # 将组添加到主布局
        self.layout.addWidget(self.data_group_box)

    def update_data(self, pos, ori):
        """更新界面上的位置和方向数据"""
        self.pos_data.setText(', '.join([f"{p:.2f}" for p in pos]))
        self.ori_data.setText(', '.join([f"{o:.2f}" for o in ori]))

    def copy_pos_data(self):
        """复制位置数据到剪贴板"""
        QApplication.clipboard().setText(self.pos_data.text())
        QMessageBox.information(self, "Copied", "Position data copied to clipboard!")

# 运行示例
if __name__ == '__main__':
    import sys
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    # 假设更新数据
    window.update_data([1.23, 4.56, 7.89], [0.123, 0.456, 0.789, 0.012])
    sys.exit(app.exec_())
