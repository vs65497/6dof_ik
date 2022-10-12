from PyQt5 import QtWidgets
from PyQt5.QtWidgets import QApplication, QMainWindow, QMenuBar, QMenu, QAction, QFileDialog
from PyQt5.QtCore import Qt, QPoint
from PyQt5.QtGui import QIcon, QImage, QPainter, QPen, QPolygon, QBrush
import sys

from time import sleep
from pathing_curvature import SmoothPath as ArmPath

class MyWindow(QMainWindow):
    def __init__(self):
        super(MyWindow, self).__init__()
        
        xpos = 100
        ypos = 50
        width = 500
        height = 500

        self.width = width
        self.height = height

        icon = "icons/ico.png"

        self.setGeometry(xpos, ypos, width, height)
        self.setWindowTitle("Pathing Simulator")
        self.setWindowIcon(QIcon(icon))
        self.initUI()

        self.run()

    def initUI(self):
        #self.label = QtWidgets.QLabel(self)
        #self.label.setText("my first label!")
        #self.label.move(100,150)

        #self.b1 = QtWidgets.QPushButton(self)
        #self.b1.setText("Click me!")
        #self.b1.move(100,100)
        #self.b1.clicked.connect(self.b1_function)

        self.image = QImage(self.size(), QImage.Format_RGB32)
        self.image.fill(Qt.white)

        self.drawing = False
        self.brushSize = 2
        self.brushColor = Qt.black

        self.lastPoint = QPoint()

        mainMenu = self.menuBar()
        fileMenu = mainMenu.addMenu("File")
        brushMenu = mainMenu.addMenu("Brush Size")
        brushColor = mainMenu.addMenu("Brush Color")

        saveAction = QAction(QIcon("icons/save.png"), "Save", self)
        saveAction.setShortcut("Ctrl+S")
        fileMenu.addAction(saveAction)
        saveAction.triggered.connect(self.save)
        
        clearAction = QAction(QIcon("icons/clear.png"), "Clear", self)
        clearAction.setShortcut("Ctrl+C")
        fileMenu.addAction(clearAction)
        clearAction.triggered.connect(self.clear)

        twopxAction = QAction(QIcon("icons/twopx.png"), "2px", self)
        twopxAction.setShortcut("Ctrl+D")
        brushMenu.addAction(twopxAction)
        twopxAction.triggered.connect(self.twoPx)
        
        threepxAction = QAction(QIcon("icons/threepx.png"), "3px", self)
        threepxAction.setShortcut("Ctrl+T")
        brushMenu.addAction(threepxAction)
        threepxAction.triggered.connect(self.threePx)
        
        fivepxAction = QAction(QIcon("icons/fivepx.png"), "5px", self)
        fivepxAction.setShortcut("Ctrl+F")
        brushMenu.addAction(fivepxAction)
        fivepxAction.triggered.connect(self.fivePx)
        
        sevenpxAction = QAction(QIcon("icons/sevenpx.png"), "7px", self)
        sevenpxAction.setShortcut("Ctrl+S")
        brushMenu.addAction(sevenpxAction)
        sevenpxAction.triggered.connect(self.sevenPx)
        
        ninepxAction = QAction(QIcon("icons/ninepx.png"), "9px", self)
        ninepxAction.setShortcut("Ctrl+N")
        brushMenu.addAction(ninepxAction)
        ninepxAction.triggered.connect(self.ninePx)
        
        blackAction = QAction(QIcon("icons/black.png"), "Black", self)
        blackAction.setShortcut("Ctrl+B")
        brushColor.addAction(blackAction)
        blackAction.triggered.connect(self.blackBrush)

        whiteAction = QAction(QIcon("icons/white.png"), "White", self)
        whiteAction.setShortcut("Ctrl+W")
        brushColor.addAction(whiteAction)
        whiteAction.triggered.connect(self.whiteBrush)

        redAction = QAction(QIcon("icons/red.png"), "Red", self)
        redAction.setShortcut("Ctrl+R")
        brushColor.addAction(redAction)
        redAction.triggered.connect(self.redBrush)

    def draw_point(self, x, y, color):
        painter = QPainter(self.image)

        x = int(x)
        y = int(y)

        painter.setPen(QPen(color, 9, Qt.SolidLine, Qt.RoundCap, Qt.RoundJoin))
        painter.drawLine(x, y, x+1, y+1)

        self.update()

    def do_draw(self, poly, x0=300, y0=300, x1=600, y1=600):
        painter = QPainter(self.image)
        
        # line
        painter.setPen(QPen(Qt.black, 2, Qt.SolidLine, Qt.RoundCap, Qt.RoundJoin))
        painter.drawLine(x0, y0, x1, y1)

        # start point
        painter.setPen(QPen(Qt.red, 9, Qt.SolidLine, Qt.RoundCap, Qt.RoundJoin))
        painter.drawLine(x0, y0, x0+1, y0+1)

        # end point
        painter.setPen(QPen(Qt.red, 9, Qt.SolidLine, Qt.RoundCap, Qt.RoundJoin))
        painter.drawLine(x1, y1, x1+1, y1+1)

        # rectangle
        painter.setPen(QPen(Qt.black, 2, Qt.SolidLine, Qt.RoundCap, Qt.RoundJoin))
        painter.drawRect(x0, y0, x1-x0, y1-y0)

        # polygon
        painter.setPen(QPen(Qt.black, 2, Qt.SolidLine, Qt.RoundCap, Qt.RoundJoin))
        painter.setBrush(QBrush(Qt.red, Qt.SolidPattern))
        points = QPolygon(poly)
        painter.drawPolygon(points)

        self.update()

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.drawing = True
            self.lastPoint = event.pos()

            print(self.lastPoint)

    def mouseMoveEvent(self, event):
        if(event.buttons() & Qt.LeftButton) & self.drawing:
            painter = QPainter(self.image)
            painter.setPen(QPen(self.brushColor, self.brushSize, Qt.SolidLine, Qt.RoundCap, Qt.RoundJoin))
            painter.drawLine(self.lastPoint, event.pos())
            self.lastPoint = event.pos()
            self.update()

    def mouseReleaseEvent(self, event):
        if event.button == Qt.LeftButton:
            self.drawing = False
    
    def paintEvent(self, event):
        canvasPainter = QPainter(self)
        canvasPainter.drawImage(self.rect(), self.image, self.image.rect())

    def save(self):
        filePath, _= QFileDialog.getSaveFileName(self, "Save Image", "", "PNG(*.png);;JPEG(*.jpg *.jpeg);; ALL Files(*.*)")
        if filePath == "":
            return
        self.image.save(filePath)

    def clear(self):
        self.image.fill(Qt.white)
        self.update()

    def twoPx(self):
        self.brushSize = 2

    def threePx(self):
        self.brushSize = 3

    def fivePx(self):
        self.brushSize = 5

    def sevenPx(self):
        self.brushSize = 7

    def ninePx(self):
        self.brushSize = 9

    def blackBrush(self):
        self.brushColor = Qt.black

    def whiteBrush(self):
        self.brushColor = Qt.white

    def redBrush(self):
        self.brushColor = Qt.red

    def b1_function(self):
        self.label.setText("you pressed the button")
        self.label.adjustSize()

        poly = [
            QPoint(200,250),
            QPoint(200,200),
            QPoint(250,200),
            QPoint(270,270)
        ]

        self.do_draw(poly)

    def run(self):

        # {"x":10,  "y":10,  "z":0},
        # {"x":15,  "y":15, "z":0},
        # {"x":20, "y":10,  "z":0}
        # IMPORTANT: this is the largest the circle can be!
        #  Need to account for ellipses!!!!!!

        points = [
            #{"x":15,  "y":15, "z":0},
            #{"x":10,  "y":10,  "z":0},
            #{"x":20, "y":10,  "z":0}
            #,
            #{"x":20,  "y":0,  "z":0},
            #{"x":25,  "y":10, "z":0},
            #{"x":30, "y":0,  "z":0}
            #[0,  10, 0],
            #[5,  20, 0],
            #[10, 10, 0],
            [15, 15, 0],
            [20, 10, 0],
            [25, 15, 0]
        ]

        print("points",points)

        resolution = 5
        max_sample_dist = -1
        trajectory = ArmPath(points, resolution, max_sample_dist)

        offset = [50,50]
        scale = self.width * 0.03

        count = 0
        for point in trajectory.path:
            #print(str(count) + ": " + str(point))
            count += 1

            #self.draw_point(point["x"]*scale+offset[0], point["y"]*scale+offset[1], Qt.red)
            self.draw_point(point[0]*scale+offset[0], point[1]*scale+offset[1], Qt.red)

        # set points
        for sp in points:
            #self.draw_point(sp["x"]*scale+offset[0], sp["y"]*scale+offset[1], Qt.blue)
            self.draw_point(sp[0]*scale+offset[0], sp[1]*scale+offset[1], Qt.blue)

def window():
    # setup
    app = QApplication(sys.argv)
    win = MyWindow()
    # necessary for showing window

    win.show()
    sys.exit(app.exec_())

window()
