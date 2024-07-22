from UI.ui_directEnvDialog import Ui_DirectEnvDialog
from PyQt5.QtWidgets import QDialog


class DirectEnvDialog(QDialog, Ui_DirectEnvDialog):
    def __init__(self):
        super().__init__()
        self.setupUi(self)
        self.closeBtn.clicked.connect(self.close)  # 关闭按钮的逻辑

        # 连接其他功能性按钮
        # self.pathPlanningBtn.clicked.connect(self.startPathPlanning)


    def test_fun1(self):
        self.infoLabel.setText('use fun1')

    # 功能性函数，如路径规划
    # def startPathPlanning(self):
    #     self.infoLabel.setText("Status: Path Planning")
