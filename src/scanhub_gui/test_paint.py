import sys
import time
import numpy as np
from PyQt5.QtWidgets import QApplication, QWidget
from PyQt5.QtGui import QPainter, QPen, QColor
from PyQt5.QtCore import Qt

class TestWidget(QWidget):
    def __init__(self):
        super().__init__()
        self.resize(800, 600)
        # 100,000 points
        self.pts = np.random.randint(0, 600, size=(100000, 2))
        
    def paintEvent(self, event):
        t_start = time.time()
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        
        pen_pts = QPen(QColor("#38d9a9"), 1.8, Qt.PenStyle.SolidLine)
        painter.setPen(pen_pts)
        
        # draw points
        for x, y in self.pts.tolist():
            painter.drawPoint(x, y)
            
        painter.end()
        print(f"Paint time: {(time.time() - t_start)*1000:.2f} ms")
        sys.exit(0)

app = QApplication(sys.argv)
w = TestWidget()
w.show()
app.exec()
