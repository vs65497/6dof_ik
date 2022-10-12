from PyQt5 import QtWidgets
from PyQt5.QtWidgets import QApplication, QMainWindow, QMenuBar, QMenu, QAction, QFileDialog
from PyQt5.QtCore import Qt, QPoint
from PyQt5.QtGui import QIcon, QImage, QPainter, QPen, QPolygon, QBrush
import sys
from threading import Timer

from linalg_graphics import Vector
from mathext import MathExt
import math

class GraphicsEngine(QMainWindow):
    def __init__(self):
        super(GraphicsEngine, self).__init__()

        self.viewport = {"w":1024/2, "h":768/2}
        self.origin = {"x":0, "y":0, "z":0}

        self.offset = {"x":self.viewport["w"]/2, "y":self.viewport["h"]/2 - self.viewport["h"]*0.05, "z":0}

        self.sphere = {}
        self.colors = {
            """ "teal": '0, 128, 128',
            "cyan": '0, 255, 255',
            "blue": '0, 0, 255',
            "purple": '128, 0, 128',
            "yellow": '255, 255, 0',
            "orange": '255, 165, 0',
            "white": '255, 255, 255',
            "black": '0, 0, 0', """
            "teal": Qt.green,
            "cyan": Qt.blue,
            "yellow": Qt.yellow,
            "white": Qt.white,
            "black": Qt.black
        }

        self.stroke_width = 1

        self.camera = {
            "direction": {"i":0, "j":0, "k":0}, # leave here for camera movement
            "fov": MathExt.to_radians(90),
            "center": {"x":0, "y":0}
        }

        self.light = {
            "center": {"x":0, "y":0, "z":0},
            "color": self.colors["white"],
            "intensity": 1
        }

        self.sphere_density = 5
        self.animation_speed = 60
        self.step_increment = 2

        self.animate_flag = False
        self.click_max_travel = 1000

        self.init_canvas()
        self.init_sphere(self.sphere_density)
        self.init_ui()
        self.update_camera()
        self.draw(self.sphere)
        
        self.show_info()
        
        # self.animate(self.sphere, self.animation_speed)

    def show_info(self):
        print('density: '+ str(self.sphere_density))
        print('animation speed: '+ str(self.animation_speed))
        print('step increment: '+ str(self.step_increment))
        print('camera:')
        print(self.camera)
        print('light source:')
        print(self.light)
        print('paint offset:')
        print(self.offset)

    def animate(self, entity, frames):
        self.animate_flag = True

        tickrate = 1 / frames
        degree = self.step_increment

        self.master_thread = Timer(tickrate, self.draw ,[entity,degree])
        self.master_thread.start()
        

    def pause(self):
        self.master_thread.cancel()

    def reset_canvas(self):
        self.image.fill(self.colors["black"])
        self.update()

    def update_camera(self):
        f = round(self.viewport["h"] / math.tan(self.camera["fov"]/2))
        self.camera["center"]["z"] = -f
        
        self.light["center"]["y"] = f
        self.light["center"]["z"] = f - 30
        # light.center.x = f + 20

    def init_canvas(self):
        xpos = 100
        ypos = 50
        width = self.viewport["w"]
        height = self.viewport["h"]

        icon = "icons/ico.png"

        self.setGeometry(xpos, ypos, width, height)
        self.setWindowTitle("3D Graphical Engine")
        self.setWindowIcon(QIcon(icon))

        self.image = QImage(self.size(), QImage.Format_RGB32)
        self.painter = QPainter(self.image)

        self.master_thread = Timer(0, None)

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.drawing = True
            self.click_angle = 0
            self.click_prev_percent = 0
            self.click_start_position = event.pos().x()

            #print(self.click_start_position)

    def mouseMoveEvent(self, event):
        if(event.buttons() & Qt.LeftButton) & self.drawing:
            cur_percent = (event.pos().x() - self.click_start_position) / self.click_max_travel
            self.click_angle = 360 * (cur_percent - self.click_prev_percent)
            self.click_prev_percent = cur_percent

            #print(self.click_angle)

            self.draw(self.sphere, self.click_angle)

    def mouseReleaseEvent(self, event):
        if event.button == Qt.LeftButton:
            self.drawing = False
    

    def init_ui(self):
        step = QtWidgets.QPushButton(self)
        step.setText("Step")
        step.move(100,self.viewport["h"]-40)
        step.clicked.connect(self.step_fun)
        
        play = QtWidgets.QPushButton(self)
        play.setText("Play")
        play.move(200,self.viewport["h"]-40)
        play.clicked.connect(self.play_fun)

        stop = QtWidgets.QPushButton(self)
        stop.setText("Stop")
        stop.move(300,self.viewport["h"]-40)
        stop.clicked.connect(self.stop_fun)

        self.image.fill(self.colors["black"])

    def play_fun(self):
        self.animate_flag = True
        self.animate(self.sphere, self.animation_speed)

    def stop_fun(self):
        self.animate_flag = False
        self.pause()

    def step_fun(self):
        if(self.animate_flag == False):
            self.draw(self.sphere,self.step_increment)

    def paintEvent(self, event):
        canvasPainter = QPainter(self)
        canvasPainter.drawImage(self.rect(), self.image, self.image.rect())

    def init_sphere(self, d):
        center = self.origin
        radius = 75
        density = 4 * d; #  quadrants * (points / quadrant)
        rotation_axis = {"w":0, "i":0.704, "j":0.71, "k":0}
        
        creation_axis = {
            "w": 0,
            "i": 0,
            "j": 1,
            "k": 0
        }
        
        #  create vertecies
        points = []
        for col in range(density):
            z = MathExt.to_radians((360/density) * (col))
            #points[col] = []
            points.append([])
            
            for row in range(density):
                x = MathExt.to_radians((360/density) * (row))
                
                r = {
                    "w": math.cos(z),
                    "x": math.sin(z)*creation_axis["i"],
                    "y": math.sin(z)*creation_axis["j"],
                    "z": math.sin(z)*creation_axis["k"]
                }

                r1 = {
                    "w": r["w"],
                    "x": -r["x"],
                    "y": -r["y"],
                    "z": -r["z"]
                }
                
                p = {
                    "w": 0,
                    "x": radius * math.sin(x),
                    "y": radius * math.cos(x),
                    "z": 0
                }

                matA = {"a": r, "b": p}
                ham_a = Vector.hamilton(**matA)
                matB = {"a": ham_a, "b": r1}
                ham_b = Vector.hamilton(**matB)
                cur_point = {
                    "x": ham_b["x"],
                    "y": ham_b["y"],
                    "z": ham_b["z"]
                }

                points[col].append(0)
                points[col][row] = Vector(cur_point) + Vector(self.origin)
        
        #  create polygons
        polygons = []
        for col in range(len(points)):
            nxc = 0 if col+1 == len(points) else col+1
            
            for row in range(len(points[col])):
                nxr = 0 if row+1 == len(points[col]) else row+1
                
                #  Leave here for texture mapping
                # rand = MathExt.get_random(0,colors.length)
                # color = colors[rand];
                # color = (color==null)? colors[0]:color;
                
                polygons.append({
                    # color: color,
                    "vertex": [
                        points[col][row],
                        points[col][nxr],
                        points[nxc][nxr],
                        points[nxc][row]
                    ]
                })
        
        self.sphere = {
            "rotation_axis": rotation_axis,
            "center": center,
            "radius": radius,
            "density": density,
            "type": 'sphere',
            "color": self.colors["cyan"],
            "polygon": polygons
        }

    def draw(self, entity, angle = 45):

        self.image.fill(Qt.black)

        f = self.camera["center"]["z"]
        c = self.camera["center"]
        
        for poly in range(len(entity["polygon"])):
            polygon = entity["polygon"][poly]
            
            coords = []
            for v in range(len(polygon["vertex"])):
                p = {}

                vec = {
                    "vector": polygon["vertex"][v],
                    "origin": entity["center"],
                    "angle": MathExt.to_radians(angle),
                    "axis": entity["rotation_axis"]
                }
                t = Vector.rotate(**vec)
                
                entity["polygon"][poly]["vertex"][v] = t

                p["x"] = (f - c["x"]) * ((t["x"] - c["x"]) / (t["z"] + f)) + c["x"]
                p["y"] = (f - c["y"]) * ((t["y"] - c["y"]) / (t["z"] + f)) + c["y"]

                coords.append(QPoint(p["x"] + self.offset["x"], p["y"] + self.offset["y"]))

            qpoly = QPolygon(coords)
            
            camera_vis = self.is_visible(self.camera["center"], entity["center"], polygon["vertex"])
            center_plane = ((camera_vis["a"])**2 + (camera_vis["b"])**2)**0.5
            backface = abs(camera_vis["c"]) < center_plane
            
            if(backface == False):
                lighting = self.light_poly(entity["center"], polygon["vertex"])

                self.painter.setPen(QPen(lighting["stroke"], self.stroke_width, lighting["stroke_style"], Qt.RoundCap, Qt.RoundJoin))
                self.painter.setBrush(QBrush(lighting["fill"], lighting["fill_style"]))

                self.painter.drawPolygon(qpoly)
                
        self.update()

        # continue animating
        if(self.animate_flag):
            self.animate(self.sphere, self.animation_speed)

    def is_visible(self, eye, center, polygon):
        v = {
            "a": Vector(center,polygon[0]),
            "b": Vector(center,polygon[3]),
            "c": Vector(center,polygon[1]),
            "d": Vector(center,polygon[2])
        }
        
        v["north"] = Vector(v["a"]) + Vector(v["b"])
        v["south"] = Vector(v["c"]) + Vector(v["d"])
        v["normal"] = Vector(v["north"]) + Vector(v["south"])

        v["to_poly"] = Vector(eye, v["normal"])
        v["to_entity"] = Vector(eye, center)
        opposite = Vector(center, v["normal"])
        
        return {
            "a": Vector.dot(v["to_entity"], eye),
            "b": Vector.dot(opposite, v["normal"]),
            "c": Vector.dot(v["to_poly"], eye)
        }

    def light_poly(self, center, polygon):
        light_vis = self.is_visible(self.light["center"], center, polygon)
        
        a = abs(light_vis["a"])
        b = abs(light_vis["b"])
        c = abs(light_vis["c"])
        
        max_c = (a**2 - b**2)**0.5
        if(type(max_c) == complex): max_c = 0
        
        fill = self.sphere["color"]
        intensity = 0
        stroke = fill

        if(c <= max_c):
            theta = 0
            if(c != 0 and b != 0):
                theta = math.atan(c / b)

            min_theta = 0
            if(max_c == 0 and b != 0):
                min_theta = math.atan(max_c / b)
            
            intensity = self.light["intensity"] - round(theta / (min_theta + .05), 1) + .1
            stroke = self.colors["white"]
        
        return {
            "intensity": intensity,
            "fill": fill,
            "stroke": stroke,
            #"fill_style": 'rgba('+fill+','+intensity+')',
            #"stroke_style": 'rgba('+stroke+', .1)'
            "fill_style": Qt.SolidPattern,
            "stroke_style": Qt.SolidLine
        }

# call the engine to do something
if __name__ == '__main__':
    app = QApplication(sys.argv)
    win = GraphicsEngine()
    # necessary for showing window
    win.show()
    sys.exit(app.exec_())