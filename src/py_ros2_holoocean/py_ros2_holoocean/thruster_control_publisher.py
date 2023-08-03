from hoveringauv_thruster_control.msg import ThrusterControl
import numpy as np
import rclpy
from rclpy.node import Node
from tkinter import *

SPIN_INTERVAL = 10
force = 50  # normally 25
thruster_command = np.zeros(8)

class ThrusterControlPublisher(Node):
    def __init__(self):
        super().__init__('thruster_control_publisher')
        self.publisher_ = self.create_publisher(ThrusterControl, 'thruster_control_topic', 3)
        self.timer = self.create_timer(0.016, self.timer_callback)  # capped 60 FPS

    def timer_callback(self):
        msg = ThrusterControl()
        msg.thruster_velocities = thruster_command
        self.publisher_.publish(msg)


def create_control_window(callback):

    def on_key_press(event):
        global thruster_command
        global force

        command = np.zeros(8)

        if event.char == 'i':
            command[0:4] += force
        if event.char == 'k':
            command[0:4] -= force
        if event.char == 'j':
            command[[4,7]] += force
            command[[5,6]] -= force
        if event.char == 'l':
            command[[4,7]] -= force
            command[[5,6]] += force

        if event.char == 'w':
            command[4:8] += force
        if event.char == 's':
            command[4:8] -= force
        if event.char == 'a':
            command[[4,6]] += force
            command[[5,7]] -= force
        if event.char == 'd':
            command[[4,6]] -= force
            command[[5,7]] += force

        thruster_command = command

    def on_key_release(event):
        global thruster_command

        thruster_command = np.zeros(8)

    root = Tk()
    root.title('Simulation Control Window')
    text = Label(root, text="\
Select this window to control the submarine:\n\
\n\
        W: move forward\n\
        A: move left\n\
        S: move backwards\n\
        D: move right\n\
        I: move up\n\
        J: turn left\n\
        K: move down\n\
        L: turn right\n\
        ", anchor="w", justify=LEFT)
    text.pack(fill=BOTH)
    root.geometry("")
    root.bind('<KeyPress>', on_key_press)
    root.bind('<KeyRelease>', on_key_release)

    def on_callback():
        callback()
        root.after(SPIN_INTERVAL, on_callback)

    root.after(SPIN_INTERVAL, on_callback)

    return root

def main(args=None):
    rclpy.init(args=args)

    thruster_control_publisher = ThrusterControlPublisher()

    def spin_thruster_control():
        rclpy.spin_once(thruster_control_publisher)

    control_window = create_control_window(spin_thruster_control)

    try:
        control_window.mainloop()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()