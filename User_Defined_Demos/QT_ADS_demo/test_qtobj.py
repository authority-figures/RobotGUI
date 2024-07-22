from PyQt5.QtCore import QObject

class SimpleTest(QObject):
    def __init__(self):
        super().__init__()

# 尝试创建SimpleTest的实例
test = SimpleTest()
