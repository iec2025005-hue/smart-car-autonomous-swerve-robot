"""
navigation/swerve_kinematics.py -- Inverse kinematics for a 4-wheel swerve drive.

Calculates the required steering angle and drive speed for each of the 4 wheel pods
based on a desired translational velocity (Vx, Vy) and rotational velocity (Omega).
"""

import math
from typing import Tuple, List

import config

class SwerveKinematics:
    def __init__(self):
        # L = distance between front and rear axles (m)
        self.L = float(config.SWERVE_WHEELBASE_LENGTH)
        if self.L > 1.0:
            self.L /= 100.0  # convert cm -> m
        # W = distance between left and right wheels (m)
        self.W = float(config.SWERVE_TRACK_WIDTH)
        if self.W > 1.0:
            self.W /= 100.0  # convert cm -> m
        self.max_speed = float(config.MAX_DRIVE_SPEED)

    def calculate(self, vx: float, vy: float, omega: float) -> List[Tuple[float, float]]:
        """
        Calculate wheel angles and speeds in REP-103 robot coordinates.
        
        Parameters
        ----------
        vx : float
            Forward velocity (+X forward).
        vy : float
            Lateral velocity (+Y left).
        omega : float
            Rotational velocity (+Z CCW).
            
        Returns
        -------
        list of (angle_deg, speed) tuples for:
            0: Front-Right (x=+L/2, y=-W/2)
            1: Front-Left  (x=+L/2, y=+W/2)
            2: Rear-Left   (x=-L/2, y=+W/2)
            3: Rear-Right  (x=-L/2, y=-W/2)
        Angles are in degrees [-180, 180] (0 = forward, +90 = left, -90 = right).
        Speeds are in m/s (or same units as vx, vy).
        """
        # FL (x = +L/2, y = +W/2): vx_fl = vx - omega*W/2, vy_fl = vy + omega*L/2
        v_fl_x = vx - omega * (self.W / 2.0)
        v_fl_y = vy + omega * (self.L / 2.0)
        s_fl = math.hypot(v_fl_x, v_fl_y)
        a_fl = math.degrees(math.atan2(v_fl_y, v_fl_x))

        # FR (x = +L/2, y = -W/2): vx_fr = vx + omega*W/2, vy_fr = vy + omega*L/2
        v_fr_x = vx + omega * (self.W / 2.0)
        v_fr_y = vy + omega * (self.L / 2.0)
        s_fr = math.hypot(v_fr_x, v_fr_y)
        a_fr = math.degrees(math.atan2(v_fr_y, v_fr_x))

        # RL (x = -L/2, y = +W/2): vx_rl = vx - omega*W/2, vy_rl = vy - omega*L/2
        v_rl_x = vx - omega * (self.W / 2.0)
        v_rl_y = vy - omega * (self.L / 2.0)
        s_rl = math.hypot(v_rl_x, v_rl_y)
        a_rl = math.degrees(math.atan2(v_rl_y, v_rl_x))

        # RR (x = -L/2, y = -W/2): vx_rr = vx + omega*W/2, vy_rr = vy - omega*L/2
        v_rr_x = vx + omega * (self.W / 2.0)
        v_rr_y = vy - omega * (self.L / 2.0)
        s_rr = math.hypot(v_rr_x, v_rr_y)
        a_rr = math.degrees(math.atan2(v_rr_y, v_rr_x))

        # Normalize speeds if any exceeds max_speed
        max_calculated = max(s_fr, s_fl, s_rl, s_rr)
        if max_calculated > self.max_speed:
            scale = self.max_speed / max_calculated
            s_fr *= scale
            s_fl *= scale
            s_rl *= scale
            s_rr *= scale

        return [
            (a_fr, s_fr),
            (a_fl, s_fl),
            (a_rl, s_rl),
            (a_rr, s_rr)
        ]


if __name__ == "__main__":
    print("Testing Swerve Kinematics")
    kinematics = SwerveKinematics()
    
    print("\n1. Forward Only (vx=100, vy=0, w=0)")
    res = kinematics.calculate(100, 0, 0)
    for i, (a, s) in enumerate(res):
        print(f"  Wheel {i}: Angle {a:5.1f}°, Speed {s:5.1f}")
        
    print("\n2. Strafe Right Only (vx=0, vy=100, w=0)")
    res = kinematics.calculate(0, 100, 0)
    for i, (a, s) in enumerate(res):
        print(f"  Wheel {i}: Angle {a:5.1f}°, Speed {s:5.1f}")
        
    print("\n3. Rotate Clockwise Only (vx=0, vy=0, w=10)")
    res = kinematics.calculate(0, 0, 10)
    for i, (a, s) in enumerate(res):
        print(f"  Wheel {i}: Angle {a:5.1f}°, Speed {s:5.1f}")
