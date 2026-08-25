import sys
import time
import numpy as np
from PyQt5.QtWidgets import QApplication, QWidget
from PyQt5.QtGui import QPainter, QPen, QColor, QPolygonF, QTransform
from PyQt5.QtCore import Qt, QPointF

class TestWidget(QWidget):
    def __init__(self):
        super().__init__()
        self.resize(800, 600)
        # 100,000 points
        pts = np.random.randint(-100, 100, size=(100000, 2))
        self.poly = QPolygonF([QPointF(float(x), float(y)) for x, y in pts.tolist()])
        
    def paintEvent(self, event):
        t_start = time.time()
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        
        pen_pts = QPen(QColor("#38d9a9"), 1.8, Qt.PenStyle.SolidLine)
        pen_pts.setCosmetic(True) # So line width isn't affected by scaling
        painter.setPen(pen_pts)
        
        cx, cy = self.width()/2, self.height()/2
        scale = 10.0
        current_x, current_y = 5.0, 5.0
        
        # Apply transforms
        painter.translate(cx, cy)
        painter.scale(-scale, -scale)
        # The original code:
        # sx = cx - ((wy - current_y) * scale)
        # sy = cy - ((wx - current_x) * scale)
        # This means X_screen = - Y_world, Y_screen = - X_world
        
        # We can set a transform explicitly:
        transform = QTransform()
        transform.translate(cx, cy)
        transform.scale(scale, scale)
        # Swap X and Y and negate them:
        # X' = -Y, Y' = -X
        transform.rotate(90)
        transform.scale(-1, 1)
        transform.translate(-current_x, -current_y)
        painter.setTransform(transform)
        
        painter.drawPoints(self.poly)
            
        painter.end()
        print(f"Paint time with Transform: {(time.time() - t_start)*1000:.2f} ms")
        sys.exit(0)

app = QApplication(sys.argv)
w = TestWidget()
w.show()
app.exec()
