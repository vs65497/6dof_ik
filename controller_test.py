from pyPS4Controller.controller import Controller

class MyController(Controller):

        def __init__(self, **kwargs):
            Controller.__init__(self, **kwargs)

        def on_x_press(self):
            print("Lift")

        def on_x_release(self):
            print("Lift halt")

        def on_circle_press(self):
            print("Lower")

        def on_circle_release(self):
            print("Lower halt")

        def on_L3_up(self, value):
            print("EXTEND")

        def on_L3_down(self, value):
            print("CONTRACT")

        def on_L3_y_at_rest(self):
            print("HALT")

controller = MyController(interface="/dev/input/js0", connecting_using_ds4drv=False)
controller.listen()
