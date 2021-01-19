import sys

from PySide2.QtUiTools import QUiLoader
from PySide2.QtCore import QFile, QEvent
from PySide2.QtWidgets import QMainWindow


class MainWindow(QMainWindow):
    """Load graphical interface layout from .ui file

    Args:
        window.ui: xml style GUIfile created from Qt Creator

    Returns:
        MainWindow: GUI data is stored in MainWindow.window
    """
    def __init__(self, ui_file, parent=None):
        super(MainWindow, self).__init__(parent)
        ui_file = QFile(ui_file)
        ui_file.open(QFile.ReadOnly)

        loader = QUiLoader()
        self.window = loader.load(ui_file)
        ui_file.close()

        self.window.show()
