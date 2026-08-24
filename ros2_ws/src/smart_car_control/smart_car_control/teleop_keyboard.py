#!/usr/bin/env python3
"""
smart_car_control/teleop_keyboard.py

Interactive keyboard teleoperation for 4-wheel swerve drive Smart Car.
Supports holonomic movement:
  W/S: Forward / Reverse
  A/D: Strafe Left / Strafe Right
  Q/E: Rotate CCW / Rotate CW
  Space or K: Stop
  +/- or X/Z: Adjust speed
"""

import sys
import termios
import tty
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

MSG = """
═══════════════════════════════════════════════════════
  Smart Car — Swerve Drive Keyboard Teleoperation
═══════════════════════════════════════════════════════
               [W] Forward
 [A] Strafe Left        [D] Strafe Right
               [S] Reverse

  [Q] Rotate Left (CCW)   [E] Rotate Right (CW)
  [Space] / [K] Emergency Stop

  [X] Increase Linear Speed (+0.1 m/s)
  [Z] Decrease Linear Speed (-0.1 m/s)
  [C] Increase Angular Speed (+0.2 rad/s)
  [V] Decrease Angular Speed (-0.2 rad/s)

  CTRL-C to exit
═══════════════════════════════════════════════════════
"""

def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def main():
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init()
    node = rclpy.create_node('teleop_keyboard_node')
    pub = node.create_publisher(Twist, '/cmd_vel', 10)

    linear_speed = 0.5   # m/s
    angular_speed = 1.0  # rad/s

    vx = 0.0
    vy = 0.0
    wz = 0.0

    print(MSG)
    print(f"Current Speeds: Linear = {linear_speed:.2f} m/s | Angular = {angular_speed:.2f} rad/s")

    try:
        while True:
            key = get_key(settings)
            if key == '\x03':  # Ctrl-C
                break

            k = key.lower()
            if k == 'w':
                vx = linear_speed
                vy = 0.0
                wz = 0.0
            elif k == 's':
                vx = -linear_speed
                vy = 0.0
                wz = 0.0
            elif k == 'a':
                vx = 0.0
                vy = linear_speed
                wz = 0.0
            elif k == 'd':
                vx = 0.0
                vy = -linear_speed
                wz = 0.0
            elif k == 'q':
                vx = 0.0
                vy = 0.0
                wz = angular_speed
            elif k == 'e':
                vx = 0.0
                vy = 0.0
                wz = -angular_speed
            elif k == ' ' or k == 'k':
                vx = 0.0
                vy = 0.0
                wz = 0.0
            elif k == 'x':
                linear_speed = min(2.0, linear_speed + 0.1)
                print(f"Linear Speed: {linear_speed:.2f} m/s")
                continue
            elif k == 'z':
                linear_speed = max(0.1, linear_speed - 0.1)
                print(f"Linear Speed: {linear_speed:.2f} m/s")
                continue
            elif k == 'c':
                angular_speed = min(3.0, angular_speed + 0.2)
                print(f"Angular Speed: {angular_speed:.2f} rad/s")
                continue
            elif k == 'v':
                angular_speed = max(0.2, angular_speed - 0.2)
                print(f"Angular Speed: {angular_speed:.2f} rad/s")
                continue
            else:
                continue

            twist = Twist()
            twist.linear.x = float(vx)
            twist.linear.y = float(vy)
            twist.angular.z = float(wz)
            pub.publish(twist)
            print(f"Publishing -> vx: {vx:+.2f} m/s | vy: {vy:+.2f} m/s | wz: {wz:+.2f} rad/s", end='\r')

    except Exception as e:
        print(f"\nError: {e}")
    finally:
        twist = Twist()
        pub.publish(twist)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()
        print("\nTeleop stopped.")


if __name__ == '__main__':
    main()
