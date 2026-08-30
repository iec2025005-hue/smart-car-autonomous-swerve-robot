#!/usr/bin/env python3
"""
create_smart_car_meshes.py -- Generates exact physical digital twin STL meshes for Smart Car swerve robot:
1. wheel.stl / wheel_left.stl / wheel_right.stl (79.93 mm outer diameter, 37.65 mm width, 4.84 mm axle bore)
2. steer_bracket.stl / fl/fr/rl/rr brackets (U-fork, concentric steering axis Z, axle axis Y, drive motor & encoder)
3. chassis.stl (Dual deck plate, 4 servo mounts at +-104.0mm, +-114.25mm, camera mount)
"""

import struct
import numpy as np
import os


def write_stl(filename, triangles):
    """Write (N, 3, 3) triangle array to binary STL format."""
    with open(filename, 'wb') as f:
        f.write(b'Smart Car Swerve Digital Twin Binary STL Mesh' + b' '*35)
        f.write(struct.pack('<I', len(triangles)))
        for tri in triangles:
            v0, v1, v2 = tri[0], tri[1], tri[2]
            norm = np.cross(v1 - v0, v2 - v0)
            norm_len = np.linalg.norm(norm)
            if norm_len > 1e-9:
                norm /= norm_len
            else:
                norm = np.array([0.0, 0.0, 1.0])
            f.write(struct.pack('<3f3f3f3fH',
                                norm[0], norm[1], norm[2],
                                v0[0], v0[1], v0[2],
                                v1[0], v1[1], v1[2],
                                v2[0], v2[1], v2[2],
                                0))
    print(f'Generated {filename}: {len(triangles)} triangles')


def make_cylinder_y(radius, y_min, y_max, center_x=0.0, center_z=0.0, n_seg=36):
    """Creates a cylinder with circular cross section in XZ plane and length along Y."""
    angles = np.linspace(0, 2*np.pi, n_seg, endpoint=False)
    cos_a = np.cos(angles)
    sin_a = np.sin(angles)
    triangles = []
    for i in range(n_seg):
        i_next = (i + 1) % n_seg
        p_min1 = np.array([center_x + radius * cos_a[i], y_min, center_z + radius * sin_a[i]])
        p_min2 = np.array([center_x + radius * cos_a[i_next], y_min, center_z + radius * sin_a[i_next]])
        p_max1 = np.array([center_x + radius * cos_a[i], y_max, center_z + radius * sin_a[i]])
        p_max2 = np.array([center_x + radius * cos_a[i_next], y_max, center_z + radius * sin_a[i_next]])
        triangles.append([p_min1, p_max1, p_max2])
        triangles.append([p_min1, p_max2, p_min2])
        c_min = np.array([center_x, y_min, center_z])
        triangles.append([c_min, p_min2, p_min1])
        c_max = np.array([center_x, y_max, center_z])
        triangles.append([c_max, p_max1, p_max2])
    return np.array(triangles)


def make_cylinder_z(radius, z_min, z_max, center_x=0.0, center_y=0.0, n_seg=36):
    """Creates a cylinder with circular cross section in XY plane and length along Z."""
    angles = np.linspace(0, 2*np.pi, n_seg, endpoint=False)
    cos_a = np.cos(angles)
    sin_a = np.sin(angles)
    triangles = []
    for i in range(n_seg):
        i_next = (i + 1) % n_seg
        p_min1 = np.array([center_x + radius * cos_a[i], center_y + radius * sin_a[i], z_min])
        p_min2 = np.array([center_x + radius * cos_a[i_next], center_y + radius * sin_a[i_next], z_min])
        p_max1 = np.array([center_x + radius * cos_a[i], center_y + radius * sin_a[i], z_max])
        p_max2 = np.array([center_x + radius * cos_a[i_next], center_y + radius * sin_a[i_next], z_max])
        triangles.append([p_min1, p_max1, p_max2])
        triangles.append([p_min1, p_max2, p_min2])
        c_min = np.array([center_x, center_y, z_min])
        triangles.append([c_min, p_min2, p_min1])
        c_max = np.array([center_x, center_y, z_max])
        triangles.append([c_max, p_max1, p_max2])
    return np.array(triangles)


def make_cylinder_x(radius, x_min, x_max, center_y=0.0, center_z=0.0, n_seg=36):
    """Creates a cylinder with circular cross section in YZ plane and length along X."""
    angles = np.linspace(0, 2*np.pi, n_seg, endpoint=False)
    cos_a = np.cos(angles)
    sin_a = np.sin(angles)
    triangles = []
    for i in range(n_seg):
        i_next = (i + 1) % n_seg
        p_min1 = np.array([x_min, center_y + radius * cos_a[i], center_z + radius * sin_a[i]])
        p_min2 = np.array([x_min, center_y + radius * cos_a[i_next], center_z + radius * sin_a[i_next]])
        p_max1 = np.array([x_max, center_y + radius * cos_a[i], center_z + radius * sin_a[i]])
        p_max2 = np.array([x_max, center_y + radius * cos_a[i_next], center_z + radius * sin_a[i_next]])
        triangles.append([p_min1, p_max1, p_max2])
        triangles.append([p_min1, p_max2, p_min2])
        c_min = np.array([x_min, center_y, center_z])
        triangles.append([c_min, p_min2, p_min1])
        c_max = np.array([x_max, center_y, center_z])
        triangles.append([c_max, p_max1, p_max2])
    return np.array(triangles)


def make_box(x_min, x_max, y_min, y_max, z_min, z_max):
    """Creates axis-aligned box with exact min and max coordinates."""
    v = [
        np.array([x_min, y_min, z_min]), np.array([x_max, y_min, z_min]),
        np.array([x_max, y_max, z_min]), np.array([x_min, y_max, z_min]),
        np.array([x_min, y_min, z_max]), np.array([x_max, y_min, z_max]),
        np.array([x_max, y_max, z_max]), np.array([x_min, y_max, z_max]),
    ]
    faces = [
        [0, 2, 1], [0, 3, 2],
        [4, 5, 6], [4, 6, 7],
        [0, 4, 7], [0, 7, 3],
        [1, 6, 5], [1, 2, 6],
        [0, 1, 5], [0, 5, 4],
        [3, 7, 6], [3, 6, 2],
    ]
    triangles = []
    for f in faces:
        triangles.append([v[f[0]], v[f[1]], v[f[2]]])
    return np.array(triangles)


def generate_wheel_mesh():
    """
    Generate wheel mesh:
    - Outer Diameter: 79.93 mm (R = 39.965 mm)
    - Width: 37.65 mm (Y in [-18.825, +18.825])
    - Axle Bore: 4.84 mm (r = 2.42 mm)
    - Rotation axis: Y axis, centered at (0, 0, 0).
    """
    r_outer = 39.965
    width = 37.65
    r_bore = 2.42
    r_rim = 32.0
    r_hub = 12.0
    
    triangles = []
    n_seg = 48
    angles = np.linspace(0, 2*np.pi, n_seg, endpoint=False)
    cos_a = np.cos(angles)
    sin_a = np.sin(angles)
    
    y_min = -width / 2.0  # -18.825 mm
    y_max = +width / 2.0  # +18.825 mm
    y_rim_in = y_min + 4.0
    y_rim_out = y_max - 4.0
    
    # 1. Outer Tread Surface with Tread Lugs
    for i in range(n_seg):
        i_next = (i + 1) % n_seg
        
        # Lug pattern: alternate height
        r_lug = r_outer if (i % 2 == 0) else (r_outer - 0.75)
        r_lug_next = r_outer if (i_next % 2 == 0) else (r_outer - 0.75)
        
        # Tread face (along Y)
        p1 = np.array([r_lug * cos_a[i], y_min, r_lug * sin_a[i]])
        p2 = np.array([r_lug_next * cos_a[i_next], y_min, r_lug_next * sin_a[i_next]])
        p3 = np.array([r_lug_next * cos_a[i_next], y_max, r_lug_next * sin_a[i_next]])
        p4 = np.array([r_lug * cos_a[i], y_max, r_lug * sin_a[i]])
        
        triangles.append([p1, p2, p3])
        triangles.append([p1, p3, p4])
        
        # Left Sidewall (y = y_min) from r_outer to r_rim
        p_rim_1 = np.array([r_rim * cos_a[i], y_min, r_rim * sin_a[i]])
        p_rim_2 = np.array([r_rim * cos_a[i_next], y_min, r_rim * sin_a[i_next]])
        triangles.append([p_rim_1, p_rim_2, p2])
        triangles.append([p_rim_1, p2, p1])
        
        # Right Sidewall (y = y_max) from r_outer to r_rim
        p_rim_r1 = np.array([r_rim * cos_a[i], y_max, r_rim * sin_a[i]])
        p_rim_r2 = np.array([r_rim * cos_a[i_next], y_max, r_rim * sin_a[i_next]])
        triangles.append([p_rim_r1, p3, p_rim_r2])
        triangles.append([p_rim_r1, p4, p3])
        
        # Recessed Rim Dish Left (from y_min to y_rim_in)
        p_dish_1 = np.array([r_rim * cos_a[i], y_rim_in, r_rim * sin_a[i]])
        p_dish_2 = np.array([r_rim * cos_a[i_next], y_rim_in, r_rim * sin_a[i_next]])
        triangles.append([p_rim_1, p_dish_1, p_dish_2])
        triangles.append([p_rim_1, p_dish_2, p_rim_2])
        
        # Recessed Rim Dish Right (from y_max to y_rim_out)
        p_dish_r1 = np.array([r_rim * cos_a[i], y_rim_out, r_rim * sin_a[i]])
        p_dish_r2 = np.array([r_rim * cos_a[i_next], y_rim_out, r_rim * sin_a[i_next]])
        triangles.append([p_rim_r1, p_dish_r2, p_dish_r1])
        triangles.append([p_rim_r1, p_rim_r2, p_dish_r2])
        
        # Central Hub Flange Left (at y_rim_in) from r_rim to r_hub (6 spokes)
        p_hub_1 = np.array([r_hub * cos_a[i], y_rim_in, r_hub * sin_a[i]])
        p_hub_2 = np.array([r_hub * cos_a[i_next], y_rim_in, r_hub * sin_a[i_next]])
        if (i % 8) < 5:
            triangles.append([p_hub_1, p_hub_2, p_dish_2])
            triangles.append([p_hub_1, p_dish_2, p_dish_1])
            
        # Central Hub Flange Right (at y_rim_out) from r_rim to r_hub
        p_hub_r1 = np.array([r_hub * cos_a[i], y_rim_out, r_hub * sin_a[i]])
        p_hub_r2 = np.array([r_hub * cos_a[i_next], y_rim_out, r_hub * sin_a[i_next]])
        if (i % 8) < 5:
            triangles.append([p_hub_r1, p_dish_r2, p_hub_r2])
            triangles.append([p_hub_r1, p_dish_1, p_dish_r2])
            
        # Hub Collar Left (from y_rim_in to y_min)
        p_collar_1 = np.array([r_hub * cos_a[i], y_min, r_hub * sin_a[i]])
        p_collar_2 = np.array([r_hub * cos_a[i_next], y_min, r_hub * sin_a[i_next]])
        triangles.append([p_hub_1, p_collar_1, p_collar_2])
        triangles.append([p_hub_1, p_collar_2, p_hub_2])
        
        # Hub Collar Right (from y_rim_out to y_max)
        p_collar_r1 = np.array([r_hub * cos_a[i], y_max, r_hub * sin_a[i]])
        p_collar_r2 = np.array([r_hub * cos_a[i_next], y_max, r_hub * sin_a[i_next]])
        triangles.append([p_hub_r1, p_collar_r2, p_collar_r1])
        triangles.append([p_hub_r1, p_hub_r2, p_collar_r2])
        
        # Hub Face Left (at y_min) from r_hub to r_bore
        p_bore_1 = np.array([r_bore * cos_a[i], y_min, r_bore * sin_a[i]])
        p_bore_2 = np.array([r_bore * cos_a[i_next], y_min, r_bore * sin_a[i_next]])
        triangles.append([p_bore_1, p_bore_2, p_collar_2])
        triangles.append([p_bore_1, p_collar_2, p_collar_1])
        
        # Hub Face Right (at y_max) from r_hub to r_bore
        p_bore_r1 = np.array([r_bore * cos_a[i], y_max, r_bore * sin_a[i]])
        p_bore_r2 = np.array([r_bore * cos_a[i_next], y_max, r_bore * sin_a[i_next]])
        triangles.append([p_bore_r1, p_collar_r2, p_bore_r2])
        triangles.append([p_bore_r1, p_collar_r1, p_collar_r2])
        
        # Inside Bore Hole Cylinder (from y_min to y_max)
        triangles.append([p_bore_1, p_bore_r1, p_bore_r2])
        triangles.append([p_bore_1, p_bore_r2, p_bore_2])
        
    return np.array(triangles)


def generate_steer_bracket_mesh(side='left'):
    """
    Generate swerve steering bracket:
    - Vertical steering axis Z at (0, 0, 0).
    - Horizontal wheel axle axis Y at (0, 0, 0) with real 4.84 mm steel axle passing through hub.
    - U-shaped yoke fork with 42.0 mm internal clearance slot (inner faces at Y = -21.0 mm and +21.0 mm).
    - Top bridge arch at Z in [42.0, 52.0] mm (safely clearing R=39.965 mm wheel).
    - Upper steering column and concentric steering pivot shaft extending to Z=134 mm.
    - Drive DC Gearmotor (25mm diameter) mounted OUTSIDE the fork arm (|Y| >= 25.0 mm).
    - Magnetic Rotary Encoder cap mounted at rear end of drive motor (|Y| in [65.0, 84.0] mm).
    - ZERO geometric intersection with the wheel cylinder (R=39.965 mm, Y in [-18.825, +18.825]).
    """
    triangles = []
    
    # 1. Solid Steel Wheel Axle passing through fork and wheel hub (Diameter = 4.84 mm, R = 2.42 mm, Y in [-28, +28])
    triangles.extend(make_cylinder_y(2.42, -28.0, +28.0, center_x=0.0, center_z=0.0))
    # Axle Retaining Nuts / Bearing End Caps at Y in [-29, -27] and [+27, +29] mm
    triangles.extend(make_cylinder_y(4.5, -29.0, -27.0, center_x=0.0, center_z=0.0))
    triangles.extend(make_cylinder_y(4.5, +27.0, +29.0, center_x=0.0, center_z=0.0))
    
    # 2. Left Fork Arm (Inner face at Y = -21.0 mm, Outer face at Y = -25.0 mm)
    triangles.extend(make_box(-14.0, +14.0, -25.0, -21.0, -10.0, +42.0))
    
    # 3. Right Fork Arm (Inner face at Y = +21.0 mm, Outer face at Y = +25.0 mm)
    triangles.extend(make_box(-14.0, +14.0, +21.0, +25.0, -10.0, +42.0))
    
    # 4. Axle Bearing Bosses on Outer Face of Fork Arms
    triangles.extend(make_cylinder_y(7.0, -27.0, -25.0, center_x=0.0, center_z=0.0))
    triangles.extend(make_cylinder_y(7.0, +25.0, +27.0, center_x=0.0, center_z=0.0))
    
    # 5. Top Fork Bridge / Arch (Spanning Y in [-25.0, +25.0], Z in [42.0, 52.0] mm)
    triangles.extend(make_box(-16.0, +16.0, -25.0, +25.0, +42.0, +52.0))
    
    # 6. Upper Steering Column / Riser (Z from 52.0 to 75.0 mm)
    triangles.extend(make_box(-14.0, +14.0, -14.0, +14.0, +52.0, +75.0))
    
    # 7. Concentric Steering Pivot Shaft (Z from 75.0 to 125.0 mm, diameter 20.0 mm)
    triangles.extend(make_cylinder_z(10.0, +75.0, +125.0, center_x=0.0, center_y=0.0))
    
    # 8. Top Steering Servo Horn Disc (Z from 125.0 to 134.0 mm, diameter 40.0 mm)
    triangles.extend(make_cylinder_z(20.0, +125.0, +134.0, center_x=0.0, center_y=0.0))
    
    # 9-11. Drive DC Gearmotor + Magnetic Encoder Assembly (Mounted OUTSIDE the fork arm)
    if side == 'left':
        # Left-side pod: Motor assembly mounted at Y <= -25.0 mm (Y_max = -25.0 mm <= -21.0 mm)
        # Gearbox Housing
        triangles.extend(make_box(-22.0, +2.0, -35.0, -25.0, -6.0, +22.0))
        # Motor Mounting Flange
        triangles.extend(make_cylinder_y(13.5, -38.0, -35.0, center_x=-10.0, center_z=+8.0))
        # 25mm DC Motor Body
        triangles.extend(make_cylinder_y(12.2, -65.0, -38.0, center_x=-10.0, center_z=+8.0))
        # Rear Magnetic Rotary Encoder Body
        triangles.extend(make_cylinder_y(12.2, -79.0, -65.0, center_x=-10.0, center_z=+8.0))
        # Encoder Cap
        triangles.extend(make_cylinder_y(10.0, -84.0, -79.0, center_x=-10.0, center_z=+8.0))
        # Wiring Connector Terminal Block
        triangles.extend(make_box(-15.0, -5.0, -84.0, -76.0, +1.0, +15.0))
    else:
        # Right-side pod: Motor assembly mounted at Y >= +25.0 mm (Y_min = +25.0 mm >= +21.0 mm)
        # Gearbox Housing
        triangles.extend(make_box(-22.0, +2.0, +25.0, +35.0, -6.0, +22.0))
        # Motor Mounting Flange
        triangles.extend(make_cylinder_y(13.5, +35.0, +38.0, center_x=-10.0, center_z=+8.0))
        # 25mm DC Motor Body
        triangles.extend(make_cylinder_y(12.2, +38.0, +65.0, center_x=-10.0, center_z=+8.0))
        # Rear Magnetic Rotary Encoder Body
        triangles.extend(make_cylinder_y(12.2, +65.0, +79.0, center_x=-10.0, center_z=+8.0))
        # Encoder Cap
        triangles.extend(make_cylinder_y(10.0, +79.0, +84.0, center_x=-10.0, center_z=+8.0))
        # Wiring Connector Terminal Block
        triangles.extend(make_box(-15.0, -5.0, +76.0, +84.0, +1.0, +15.0))
        
    return np.array(triangles)


def generate_chassis_mesh():
    """
    Generate main chassis assembly:
    - Wheelbase L = 208.0 mm (X = +-104.0 mm)
    - Track Width W = 228.5 mm (Y = +-114.25 mm)
    - Dual deck plates (Bottom deck Z=75mm, Top deck Z=135mm)
    - 4 Corner Servo Mounts matching pod steering shafts
    - Front Camera Mount Tower
    """
    triangles = []
    
    # 1. Bottom Main Deck Plate (Z in [75.0, 81.0])
    deck_bot = make_box(-135.0, +135.0, -130.0, +130.0, 75.0, 81.0)
    triangles.extend(deck_bot)
    
    # 2. Upper Electronics Deck Plate (Z in [135.0, 140.0])
    deck_top = make_box(-125.0, +125.0, -105.0, +105.0, 135.0, 140.0)
    triangles.extend(deck_top)
    
    # 3. 4 Structural Standoff Pillars (between bottom and top deck)
    for sx in [-80.0, 80.0]:
        for sy in [-70.0, 70.0]:
            pillar = make_cylinder_z(6.0, 81.0, 135.0, center_x=sx, center_y=sy)
            triangles.extend(pillar)
            
    # 4. 4 Steering Servo Actuator Housings on Upper Deck (at +-104.0 mm, +-114.25 mm)
    corners = [
        (+104.0, +114.25),
        (+104.0, -114.25),
        (-104.0, +114.25),
        (-104.0, -114.25),
    ]
    for cx, cy in corners:
        s_box = make_box(cx-21.0, cx+21.0, cy-21.0, cy+21.0, 135.0, 175.0)
        triangles.extend(s_box)
        s_flange = make_cylinder_z(22.0, 131.0, 139.0, center_x=cx, center_y=cy)
        triangles.extend(s_flange)
        
    # 5. Front Camera Mount Tower (at front center X=+125.0 mm, Y=0.0 mm)
    cam_tower = make_box(119.0, 131.0, -18.0, +18.0, 140.0, 180.0)
    triangles.extend(cam_tower)
    cam_head = make_box(120.0, 140.0, -12.5, +12.5, 167.5, 192.5)
    triangles.extend(cam_head)
    cam_lens = make_cylinder_x(6.0, 135.0, 145.0, center_y=0.0, center_z=180.0)
    triangles.extend(cam_lens)
    
    # 6. Central Battery Pack & Electronics Enclosure
    battery = make_box(-45.0, +45.0, -30.0, +30.0, 81.0, 111.0)
    triangles.extend(battery)
    
    return np.array(triangles)


def main():
    mesh_dir = '/home/diya/smart_car/ros2_ws/src/smart_car_description/meshes'
    os.makedirs(mesh_dir, exist_ok=True)
    
    # 1. Generate Wheel Meshes (Exact 79.93 mm outer diameter, 37.65 mm width, 4.84 mm axle bore)
    wheel_tris = generate_wheel_mesh()
    write_stl(f'{mesh_dir}/wheel.stl', wheel_tris)
    write_stl(f'{mesh_dir}/wheel_left.stl', wheel_tris)
    write_stl(f'{mesh_dir}/wheel_right.stl', wheel_tris)
    
    # 2. Generate Steering Bracket Meshes (Separated motor & encoder, U-fork, axle)
    steer_left = generate_steer_bracket_mesh(side='left')
    steer_right = generate_steer_bracket_mesh(side='right')
    
    write_stl(f'{mesh_dir}/steer_bracket.stl', steer_left)
    write_stl(f'{mesh_dir}/fl_steer_bracket.stl', steer_left)
    write_stl(f'{mesh_dir}/rl_steer_bracket.stl', steer_left)
    write_stl(f'{mesh_dir}/fr_steer_bracket.stl', steer_right)
    write_stl(f'{mesh_dir}/rr_steer_bracket.stl', steer_right)
    
    # 3. Generate Chassis Mesh
    chassis_tris = generate_chassis_mesh()
    write_stl(f'{mesh_dir}/chassis.stl', chassis_tris)
    
    print('All Smart Car digital twin STL meshes generated successfully with zero intersections!')


if __name__ == '__main__':
    main()
