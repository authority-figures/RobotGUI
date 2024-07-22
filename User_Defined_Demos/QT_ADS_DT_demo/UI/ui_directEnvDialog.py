from PyQt5.QtWidgets import QDialog, QPushButton, QVBoxLayout, QLabel

class Ui_DirectEnvDialog(object):
    def setupUi(self, Dialog):
        Dialog.setWindowTitle("DIRECT Environment Tools")
        Dialog.resize(300, 200)

        self.layout = QVBoxLayout(Dialog)

        self.pathPlanningBtn = QPushButton("Start Path Planning", Dialog)
        self.layout.addWidget(self.pathPlanningBtn)

        self.infoLabel = QLabel("Status: Idle", Dialog)
        self.layout.addWidget(self.infoLabel)

        self.closeBtn = QPushButton("Close", Dialog)
        self.layout.addWidget(self.closeBtn)
