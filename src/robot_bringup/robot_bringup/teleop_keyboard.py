#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty
import select
import time


class TeleopKeyboard(Node):

    def __init__(self):
        super().__init__('teleop_keyboard')

        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        # SLAM-friendly speeds
        self.linear_speed = 0.25
        self.angular_speed = 0.7

        self.timeout = 0.1  # seconds (auto-stop if no key)

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], self.timeout)

        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

    def run(self):

        print("""
==============================
  TELEOP CONTROL (SLAM READY)
==============================

Move:
   w    forward
   s    backward
   a    turn left
   d    turn right

Space : STOP
q      : Quit

==============================
""")

        twist = Twist()

        while rclpy.ok():

            key = self.get_key()

            if key == 'w':
                twist.linear.x = self.linear_speed
                twist.angular.z = 0.0

            elif key == 's':
                twist.linear.x = -self.linear_speed
                twist.angular.z = 0.0

            elif key == 'a':
                twist.linear.x = 0.0
                twist.angular.z = self.angular_speed

            elif key == 'd':
                twist.linear.x = 0.0
                twist.angular.z = -self.angular_speed

            elif key == ' ':
                twist = Twist()

            elif key == 'q':
                print("Exiting teleop...")
                break

            else:
                # Auto stop if no valid key
                twist = Twist()

            self.publisher_.publish(twist)

        # Ensure robot stops on exit
        self.publisher_.publish(Twist())


def main(args=None):
    global settings
    settings = termios.tcgetattr(sys.stdin)

    rclpy.init(args=args)
    teleop = TeleopKeyboard()

    try:
        teleop.run()
    except Exception as e:
        print(e)
    finally:
        teleop.destroy_node()
        rclpy.shutdown()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


if __name__ == '__main__':
    main()
