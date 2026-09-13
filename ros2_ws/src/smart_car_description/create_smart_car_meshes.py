#!/usr/bin/env python3
"""
create_smart_car_meshes.py -- Generates exact physical digital twin STL and DAE (COLLADA) meshes
for Smart Car swerve robot natively in METERS:
1. wheel.dae / wheel.stl (79.93 mm outer diameter, 37.65 mm width, 4.84 mm axle bore)
   - Distinct materials: Rubber tire with tread lugs, Silver alloy rim with 6 spokes, Steel bore.
2. fl/fr/rl/rr/steer_bracket.dae & .stl (U-fork, concentric steering axis Z, axle axis Y)
   - Distinct materials: Dark anodized fork, Polished steel axle, Brushed gold DC motor, Red magnetic encoder.
   - Motors and encoders are positioned strictly OUTBOARD of the wheels (Left at +Y, Right at -Y).
3. chassis.dae & .stl (Dual deck plate, 4 servo mounts, aluminum standoffs, camera tower)
   - Distinct materials: Dark carbon decks, Silver aluminum pillars, Black servos, Blue battery, Green camera.

All coordinates are strictly in SI METERS.
DAE headers declare <unit name="meter" meter="1.0"/> and <up_axis>Y_UP</up_axis> ensuring
Assimp/RViz2/Gazebo load them with an identity transform without hidden scaling or axis swaps.
"""

import struct
import numpy as np
import os


def write_stl(filename, triangles):
    """Write (N, 3, 3) triangle array to binary STL format in meters."""
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
    print(f'Generated STL {filename}: {len(triangles)} triangles')


def write_dae(filename, parts):
    """
    Write multi-part, multi-material COLLADA 1.4.1 (.dae) mesh natively in METERS.
    parts: list of dicts:
      - 'id': str
      - 'name': str
      - 'color': (r, g, b, a) float
      - 'triangles': (N, 3, 3) np.ndarray (in meters)
    """
    out = ['<?xml version="1.0" encoding="utf-8"?>']
    out.append('<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">')
    out.append('  <asset>')
    out.append('    <contributor><authoring_tool>SmartCar CAD Digital Twin</authoring_tool></contributor>')
    out.append('    <created>2026-09-01T00:00:00</created>')
    out.append('    <modified>2026-09-01T00:00:00</modified>')
    out.append('    <unit name="meter" meter="1.0"/>')
    out.append('    <up_axis>Y_UP</up_axis>')
    out.append('  </asset>')

    # 1. Effects
    out.append('  <library_effects>')
    for p in parts:
        pid = p['id']
        r, g, b, a = p['color']
        spec = p.get('specular', (0.4, 0.4, 0.4, 1.0))
        sr, sg, sb, sa = spec
        shin = p.get('shininess', 30.0)
        out.append(f'    <effect id="{pid}-effect">')
        out.append('      <profile_COMMON>')
        out.append('        <technique sid="common">')
        out.append('          <phong>')
        out.append('            <emission><color sid="emission">0 0 0 1</color></emission>')
        out.append(f'            <ambient><color sid="ambient">{r*0.25:.3f} {g*0.25:.3f} {b*0.25:.3f} 1</color></ambient>')
        out.append(f'            <diffuse><color sid="diffuse">{r:.3f} {g:.3f} {b:.3f} {a:.3f}</color></diffuse>')
        out.append(f'            <specular><color sid="specular">{sr:.3f} {sg:.3f} {sb:.3f} {sa:.3f}</color></specular>')
        out.append(f'            <shininess><float sid="shininess">{shin:.1f}</float></shininess>')
        out.append('          </phong>')
        out.append('        </technique>')
        out.append('      </profile_COMMON>')
        out.append('    </effect>')
    out.append('  </library_effects>')

    # 2. Materials
    out.append('  <library_materials>')
    for p in parts:
        pid = p['id']
        pname = p['name']
        out.append(f'    <material id="{pid}-mat" name="{pname}">')
        out.append(f'      <instance_effect url="#{pid}-effect"/>')
        out.append('    </material>')
    out.append('  </library_materials>')

    # 3. Geometries
    out.append('  <library_geometries>')
    for p in parts:
        pid = p['id']
        tris = p['triangles']
        num_tri = len(tris)
        if num_tri == 0:
            continue

        verts = tris.reshape(-1, 3)
        norms = np.zeros_like(verts)
        for i in range(num_tri):
            v0, v1, v2 = tris[i]
            n = np.cross(v1 - v0, v2 - v0)
            nl = np.linalg.norm(n)
            n = n / nl if nl > 1e-9 else np.array([0.0, 0.0, 1.0])
            norms[i*3] = n
            norms[i*3 + 1] = n
            norms[i*3 + 2] = n

        pos_str = ' '.join(f'{x:.6f}' for x in verts.flatten())
        norm_str = ' '.join(f'{x:.4f}' for x in norms.flatten())
        indices = np.arange(len(verts))
        p_pairs = np.column_stack((indices, indices)).flatten()
        p_str = ' '.join(str(x) for x in p_pairs)

        out.append(f'    <geometry id="{pid}-geom" name="{pid}">')
        out.append('      <mesh>')
        out.append(f'        <source id="{pid}-pos">')
        out.append(f'          <float_array id="{pid}-pos-array" count="{len(verts)*3}">{pos_str}</float_array>')
        out.append('          <technique_common>')
        out.append(f'            <accessor source="#{pid}-pos-array" count="{len(verts)}" stride="3">')
        out.append('              <param name="X" type="float"/>')
        out.append('              <param name="Y" type="float"/>')
        out.append('              <param name="Z" type="float"/>')
        out.append('            </accessor>')
        out.append('          </technique_common>')
        out.append('        </source>')
        out.append(f'        <source id="{pid}-nrm">')
        out.append(f'          <float_array id="{pid}-nrm-array" count="{len(norms)*3}">{norm_str}</float_array>')
        out.append('          <technique_common>')
        out.append(f'            <accessor source="#{pid}-nrm-array" count="{len(norms)}" stride="3">')
        out.append('              <param name="X" type="float"/>')
        out.append('              <param name="Y" type="float"/>')
        out.append('              <param name="Z" type="float"/>')
        out.append('            </accessor>')
        out.append('          </technique_common>')
        out.append('        </source>')
        out.append(f'        <vertices id="{pid}-vtx">')
        out.append(f'          <input semantic="POSITION" source="#{pid}-pos"/>')
        out.append('        </vertices>')
        out.append(f'        <triangles material="{pid}-mat" count="{num_tri}">')
        out.append(f'          <input semantic="VERTEX" source="#{pid}-vtx" offset="0"/>')
        out.append(f'          <input semantic="NORMAL" source="#{pid}-nrm" offset="1"/>')
        out.append(f'          <p>{p_str}</p>')
        out.append('        </triangles>')
        out.append('      </mesh>')
        out.append('    </geometry>')
    out.append('  </library_geometries>')

    # 4. Visual Scene
    out.append('  <library_visual_scenes>')
    out.append('    <visual_scene id="Scene" name="Scene">')
    out.append('      <node id="RootNode" name="RootNode" type="NODE">')
    for p in parts:
        pid = p['id']
        if len(p['triangles']) == 0:
            continue
        out.append(f'        <instance_geometry url="#{pid}-geom">')
        out.append('          <bind_material>')
        out.append('            <technique_common>')
        out.append(f'              <instance_material symbol="{pid}-mat" target="#{pid}-mat"/>')
        out.append('            </technique_common>')
        out.append('          </bind_material>')
        out.append('        </instance_geometry>')
    out.append('      </node>')
    out.append('    </visual_scene>')
    out.append('  </library_visual_scenes>')
    out.append('  <scene>')
    out.append('    <instance_visual_scene url="#Scene"/>')
    out.append('  </scene>')
    out.append('</COLLADA>')

    with open(filename, 'w', encoding='utf-8') as f:
        f.write('\n'.join(out))
    print(f"Generated DAE {filename}: {len(parts)} parts, {sum(len(p['triangles']) for p in parts)} triangles")


def make_cylinder_y(radius, y_min, y_max, center_x=0.0, center_z=0.0, n_seg=36):
    """Creates a cylinder with circular cross section in XZ plane and length along Y in meters."""
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
    """Creates a cylinder with circular cross section in XY plane and length along Z in meters."""
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
    """Creates a cylinder with circular cross section in YZ plane and length along X in meters."""
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
    """Creates axis-aligned box with exact min and max coordinates in meters."""
    v = [
        np.array([x_min, y_min, z_min]), np.array([x_max, y_min, z_min]),
        np.array([x_max, y_max, z_min]), np.array([x_min, y_max, z_min]),
        np.array([x_min, y_min, z_max]), np.array([x_max, y_min, z_max]),
        np.array([x_max, y_max, z_max]), np.array([x_min, y_max, z_max]),
    ]
    f = [
        (0, 1, 2), (0, 2, 3),  # bottom (-Z)
        (4, 6, 5), (4, 7, 6),  # top (+Z)
        (0, 4, 5), (0, 5, 1),  # front (-Y)
        (2, 6, 7), (2, 7, 3),  # back (+Y)
        (0, 3, 7), (0, 7, 4),  # left (-X)
        (1, 5, 6), (1, 6, 2),  # right (+X)
    ]
    return np.array([[v[i], v[j], v[k]] for i, j, k in f])


def generate_wheel_parts():
    """
    Generate wheel parts natively in METERS:
    - Outer Diameter: 79.93 mm = 0.07993 m (Radius = 0.039965 m)
    - Width: 37.65 mm = 0.03765 m (Y in [-0.018825, +0.018825] m)
    - Axle Bore: 4.84 mm = 0.00484 m (Radius = 0.00242 m)
    """
    r_outer = 0.039965  # 39.965 mm
    width = 0.03765     # 37.65 mm
    r_bore = 0.00242    # 2.42 mm (D = 4.84 mm)
    r_rim = 0.0320      # 32.0 mm
    r_hub = 0.0120      # 12.0 mm
    
    tire_tris = []
    rim_tris = []
    bore_tris = []
    
    n_seg = 48
    angles = np.linspace(0, 2*np.pi, n_seg, endpoint=False)
    cos_a = np.cos(angles)
    sin_a = np.sin(angles)
    
    y_min = -width / 2.0  # -0.018825 m
    y_max = +width / 2.0  # +0.018825 m
    y_rim_in = y_min + 0.0040
    y_rim_out = y_max - 0.0040
    
    for i in range(n_seg):
        i_next = (i + 1) % n_seg
        r_lug = r_outer if (i % 2 == 0) else (r_outer - 0.00075)
        r_lug_next = r_outer if (i_next % 2 == 0) else (r_outer - 0.00075)
        
        # 1. Tire Tread Lugs
        p1 = np.array([r_lug * cos_a[i], y_min, r_lug * sin_a[i]])
        p2 = np.array([r_lug_next * cos_a[i_next], y_min, r_lug_next * sin_a[i_next]])
        p3 = np.array([r_lug_next * cos_a[i_next], y_max, r_lug_next * sin_a[i_next]])
        p4 = np.array([r_lug * cos_a[i], y_max, r_lug * sin_a[i]])
        tire_tris.append([p1, p2, p3])
        tire_tris.append([p1, p3, p4])
        
        # 2. Tire Sidewalls
        p_rim_1 = np.array([r_rim * cos_a[i], y_min, r_rim * sin_a[i]])
        p_rim_2 = np.array([r_rim * cos_a[i_next], y_min, r_rim * sin_a[i_next]])
        tire_tris.append([p_rim_1, p_rim_2, p2])
        tire_tris.append([p_rim_1, p2, p1])
        
        p_rim_r1 = np.array([r_rim * cos_a[i], y_max, r_rim * sin_a[i]])
        p_rim_r2 = np.array([r_rim * cos_a[i_next], y_max, r_rim * sin_a[i_next]])
        tire_tris.append([p_rim_r1, p3, p_rim_r2])
        tire_tris.append([p_rim_r1, p4, p3])
        
        # 3. Metallic Rim Dish
        p_dish_1 = np.array([r_rim * cos_a[i], y_rim_in, r_rim * sin_a[i]])
        p_dish_2 = np.array([r_rim * cos_a[i_next], y_rim_in, r_rim * sin_a[i_next]])
        rim_tris.append([p_rim_1, p_dish_1, p_dish_2])
        rim_tris.append([p_rim_1, p_dish_2, p_rim_2])
        
        p_dish_r1 = np.array([r_rim * cos_a[i], y_rim_out, r_rim * sin_a[i]])
        p_dish_r2 = np.array([r_rim * cos_a[i_next], y_rim_out, r_rim * sin_a[i_next]])
        rim_tris.append([p_rim_r1, p_dish_r2, p_dish_r1])
        rim_tris.append([p_rim_r1, p_rim_r2, p_dish_r2])
        
        # 4. Spokes and Hub
        p_hub_1 = np.array([r_hub * cos_a[i], y_rim_in, r_hub * sin_a[i]])
        p_hub_2 = np.array([r_hub * cos_a[i_next], y_rim_in, r_hub * sin_a[i_next]])
        if (i % 8) < 5:
            rim_tris.append([p_hub_1, p_hub_2, p_dish_2])
            rim_tris.append([p_hub_1, p_dish_2, p_dish_1])
            
        p_hub_r1 = np.array([r_hub * cos_a[i], y_rim_out, r_hub * sin_a[i]])
        p_hub_r2 = np.array([r_hub * cos_a[i_next], y_rim_out, r_hub * sin_a[i_next]])
        if (i % 8) < 5:
            rim_tris.append([p_hub_r1, p_dish_r2, p_hub_r2])
            rim_tris.append([p_hub_r1, p_dish_1, p_dish_r2])
            
        # Hub Collars
        p_collar_1 = np.array([r_hub * cos_a[i], y_min, r_hub * sin_a[i]])
        p_collar_2 = np.array([r_hub * cos_a[i_next], y_min, r_hub * sin_a[i_next]])
        rim_tris.append([p_hub_1, p_collar_1, p_collar_2])
        rim_tris.append([p_hub_1, p_collar_2, p_hub_2])
        
        p_collar_r1 = np.array([r_hub * cos_a[i], y_max, r_hub * sin_a[i]])
        p_collar_r2 = np.array([r_hub * cos_a[i_next], y_max, r_hub * sin_a[i_next]])
        rim_tris.append([p_hub_r1, p_collar_r2, p_collar_r1])
        rim_tris.append([p_hub_r1, p_hub_r2, p_collar_r2])
        
        # Bore Face
        p_bore_1 = np.array([r_bore * cos_a[i], y_min, r_bore * sin_a[i]])
        p_bore_2 = np.array([r_bore * cos_a[i_next], y_min, r_bore * sin_a[i_next]])
        bore_tris.append([p_bore_1, p_bore_2, p_collar_2])
        bore_tris.append([p_bore_1, p_collar_2, p_collar_1])
        
        p_bore_r1 = np.array([r_bore * cos_a[i], y_max, r_bore * sin_a[i]])
        p_bore_r2 = np.array([r_bore * cos_a[i_next], y_max, r_bore * sin_a[i_next]])
        bore_tris.append([p_bore_r1, p_collar_r2, p_bore_r2])
        bore_tris.append([p_bore_r1, p_collar_1, p_collar_2])
        
        # 5. Inside Bore Cylinder
        bore_tris.append([p_bore_1, p_bore_r1, p_bore_r2])
        bore_tris.append([p_bore_1, p_bore_r2, p_bore_2])

    parts = [
        {
            'id': 'tire_rubber',
            'name': 'TireRubber',
            'color': (0.13, 0.13, 0.14, 1.0),
            'specular': (0.1, 0.1, 0.1, 1.0),
            'shininess': 10.0,
            'triangles': np.array(tire_tris)
        },
        {
            'id': 'wheel_rim',
            'name': 'SilverRim',
            'color': (0.75, 0.76, 0.78, 1.0),
            'specular': (0.7, 0.7, 0.75, 1.0),
            'shininess': 50.0,
            'triangles': np.array(rim_tris)
        },
        {
            'id': 'axle_bore',
            'name': 'AxleBore',
            'color': (0.50, 0.52, 0.55, 1.0),
            'specular': (0.5, 0.5, 0.5, 1.0),
            'shininess': 40.0,
            'triangles': np.array(bore_tris)
        }
    ]
    return parts


def generate_steer_bracket_parts(side='left'):
    """
    Generate swerve steering bracket natively in METERS:
    - Fork arm and bridge: Dark Anodized Slate (#2c3e50)
    - Axle & nuts: Polished Stainless Steel (#ecf0f1)
    - Pivot shaft & horn: Matte Charcoal Steel (#34495e)
    - Gearbox housing & flange: Gunmetal Grey (#455a64)
    - DC Gearmotor body: Brushed Gold/Brass (#d4af37)
    - Magnetic encoder body: Racing Red (#c0392b)
    - Encoder cap & terminal: Composite Black & White (#1e272e)
    All motors sit strictly OUTBOARD (Left at +Y, Right at -Y).
    """
    fork_tris = []
    axle_tris = []
    pivot_tris = []
    gearbox_tris = []
    motor_tris = []
    encoder_tris = []

    # 1. Solid Steel Wheel Axle (D = 4.84 mm, R = 2.42 mm, Y in [-0.028, +0.028] m)
    axle_tris.extend(make_cylinder_y(0.00242, -0.028, +0.028, center_x=0.0, center_z=0.0))
    axle_tris.extend(make_cylinder_y(0.0045, -0.029, -0.027, center_x=0.0, center_z=0.0))
    axle_tris.extend(make_cylinder_y(0.0045, +0.027, +0.029, center_x=0.0, center_z=0.0))

    # 2. Fork Arms (Inner faces at Y = +-0.021 m (42 mm gap), Outer faces at Y = +-0.025 m)
    fork_tris.extend(make_box(-0.014, +0.014, -0.025, -0.021, -0.010, +0.042))
    fork_tris.extend(make_box(-0.014, +0.014, +0.021, +0.025, -0.010, +0.042))
    fork_tris.extend(make_cylinder_y(0.007, -0.027, -0.025, center_x=0.0, center_z=0.0))
    fork_tris.extend(make_cylinder_y(0.007, +0.025, +0.027, center_x=0.0, center_z=0.0))

    # 3. Top Fork Bridge & Steering Column (Z in [0.042, 0.075] m)
    fork_tris.extend(make_box(-0.016, +0.016, -0.025, +0.025, +0.042, +0.052))
    fork_tris.extend(make_box(-0.014, +0.014, -0.014, +0.014, +0.052, +0.075))

    # 4. Concentric Steering Pivot Shaft & Horn Disc (Z in [0.075, 0.134] m)
    pivot_tris.extend(make_cylinder_z(0.010, +0.075, +0.125, center_x=0.0, center_y=0.0))
    pivot_tris.extend(make_cylinder_z(0.020, +0.125, +0.134, center_x=0.0, center_y=0.0))

    # 5. Outboard Motor, Gearbox, and Encoder Assembly
    if side == 'left':
        # Outboard is +Y (away from car centerline)
        gearbox_tris.extend(make_box(-0.022, +0.002, +0.025, +0.035, -0.006, +0.022))
        gearbox_tris.extend(make_cylinder_y(0.0135, +0.035, +0.038, center_x=-0.010, center_z=+0.008))
        motor_tris.extend(make_cylinder_y(0.0122, +0.038, +0.065, center_x=-0.010, center_z=+0.008))
        encoder_tris.extend(make_cylinder_y(0.0122, +0.065, +0.079, center_x=-0.010, center_z=+0.008))
        encoder_tris.extend(make_cylinder_y(0.0100, +0.079, +0.084, center_x=-0.010, center_z=+0.008))
        encoder_tris.extend(make_box(-0.015, -0.005, +0.076, +0.084, +0.001, +0.015))
    else:
        # Outboard is -Y (away from car centerline)
        gearbox_tris.extend(make_box(-0.022, +0.002, -0.035, -0.025, -0.006, +0.022))
        gearbox_tris.extend(make_cylinder_y(0.0135, -0.038, -0.035, center_x=-0.010, center_z=+0.008))
        motor_tris.extend(make_cylinder_y(0.0122, -0.065, -0.038, center_x=-0.010, center_z=+0.008))
        encoder_tris.extend(make_cylinder_y(0.0122, -0.079, -0.065, center_x=-0.010, center_z=+0.008))
        encoder_tris.extend(make_cylinder_y(0.0100, -0.084, -0.079, center_x=-0.010, center_z=+0.008))
        encoder_tris.extend(make_box(-0.015, -0.005, -0.084, -0.076, +0.001, +0.015))

    parts = [
        {
            'id': f'{side}_fork',
            'name': 'ForkBracket',
            'color': (0.18, 0.22, 0.28, 1.0),
            'specular': (0.3, 0.35, 0.4, 1.0),
            'shininess': 25.0,
            'triangles': np.array(fork_tris)
        },
        {
            'id': f'{side}_axle',
            'name': 'WheelAxle',
            'color': (0.85, 0.88, 0.90, 1.0),
            'specular': (0.8, 0.8, 0.85, 1.0),
            'shininess': 60.0,
            'triangles': np.array(axle_tris)
        },
        {
            'id': f'{side}_pivot',
            'name': 'SteerPivot',
            'color': (0.22, 0.25, 0.30, 1.0),
            'specular': (0.4, 0.4, 0.45, 1.0),
            'shininess': 30.0,
            'triangles': np.array(pivot_tris)
        },
        {
            'id': f'{side}_gearbox',
            'name': 'GearboxHousing',
            'color': (0.32, 0.36, 0.40, 1.0),
            'specular': (0.5, 0.5, 0.55, 1.0),
            'shininess': 35.0,
            'triangles': np.array(gearbox_tris)
        },
        {
            'id': f'{side}_motor',
            'name': 'DCMotorBody',
            'color': (0.78, 0.62, 0.22, 1.0),  # Brushed Gold / Brass
            'specular': (0.7, 0.6, 0.3, 1.0),
            'shininess': 45.0,
            'triangles': np.array(motor_tris)
        },
        {
            'id': f'{side}_encoder',
            'name': 'EncoderHousing',
            'color': (0.80, 0.15, 0.15, 1.0),  # Racing Red
            'specular': (0.5, 0.2, 0.2, 1.0),
            'shininess': 30.0,
            'triangles': np.array(encoder_tris)
        }
    ]
    return parts


def generate_chassis_parts():
    """
    Generate chassis parts natively in METERS:
    - Deck plates: Carbon Fiber Dark Slate (#212529)
    - Pillars: Brushed Aluminum Silver (#ced4da)
    - Servos: Jet Black Actuators (#121212)
    - Battery: Industrial Blue (#1976d2)
    - Camera Tower: Matte Black (#212121)
    - Camera Head: Green PCB (#2e7d32) & Optical Lens (#0d47a1)
    """
    deck_tris = []
    pillar_tris = []
    servo_tris = []
    battery_tris = []
    camera_tris = []

    # 1. Main Decks (Bottom Z in [0.075, 0.081], Top Z in [0.135, 0.140] m)
    deck_tris.extend(make_box(-0.135, +0.135, -0.130, +0.130, 0.075, 0.081))
    deck_tris.extend(make_box(-0.125, +0.125, -0.105, +0.105, 0.135, 0.140))

    # 2. 4 Structural Standoff Pillars
    for sx in [-0.080, 0.080]:
        for sy in [-0.070, 0.070]:
            pillar_tris.extend(make_cylinder_z(0.006, 0.081, 0.135, center_x=sx, center_y=sy))

    # 3. 4 Steering Servos on Upper Deck (at +-0.104 m, +-0.11425 m)
    corners = [(+0.104, +0.11425), (+0.104, -0.11425), (-0.104, +0.11425), (-0.104, -0.11425)]
    for cx, cy in corners:
        servo_tris.extend(make_box(cx-0.021, cx+0.021, cy-0.021, cy+0.021, 0.135, 0.175))
        servo_tris.extend(make_cylinder_z(0.022, 0.131, 0.139, center_x=cx, center_y=cy))

    # 4. Front Camera Mount Tower (X=+0.125 m, optical center Z=+0.180 m)
    camera_tris.extend(make_box(0.119, 0.131, -0.018, +0.018, 0.140, 0.180))
    camera_tris.extend(make_box(0.120, 0.140, -0.0125, +0.0125, 0.1675, 0.1925))
    camera_tris.extend(make_cylinder_x(0.006, 0.135, 0.145, center_y=0.0, center_z=0.180))

    # 5. Central Battery Pack
    battery_tris.extend(make_box(-0.045, +0.045, -0.030, +0.030, 0.081, 0.111))

    parts = [
        {
            'id': 'chassis_decks',
            'name': 'ChassisPlates',
            'color': (0.16, 0.18, 0.22, 1.0),
            'specular': (0.25, 0.25, 0.3, 1.0),
            'shininess': 20.0,
            'triangles': np.array(deck_tris)
        },
        {
            'id': 'chassis_pillars',
            'name': 'AluminumStandoffs',
            'color': (0.82, 0.84, 0.87, 1.0),
            'specular': (0.75, 0.75, 0.8, 1.0),
            'shininess': 55.0,
            'triangles': np.array(pillar_tris)
        },
        {
            'id': 'chassis_servos',
            'name': 'SteeringServos',
            'color': (0.10, 0.10, 0.12, 1.0),
            'specular': (0.2, 0.2, 0.2, 1.0),
            'shininess': 20.0,
            'triangles': np.array(servo_tris)
        },
        {
            'id': 'chassis_battery',
            'name': 'BatteryPack',
            'color': (0.12, 0.40, 0.78, 1.0),
            'specular': (0.4, 0.5, 0.7, 1.0),
            'shininess': 30.0,
            'triangles': np.array(battery_tris)
        },
        {
            'id': 'chassis_camera',
            'name': 'CameraTower',
            'color': (0.15, 0.45, 0.25, 1.0),
            'specular': (0.5, 0.7, 0.5, 1.0),
            'shininess': 40.0,
            'triangles': np.array(camera_tris)
        }
    ]
    return parts


def main():
    mesh_dir = '/home/diya/smart_car/ros2_ws/src/smart_car_description/meshes'
    os.makedirs(mesh_dir, exist_ok=True)
    
    print('Generating exact digital twin meshes (DAE + STL) in METERS...')

    # 1. Wheels (Exact 79.93 mm outer diameter, 37.65 mm width, 4.84 mm axle bore)
    wheel_parts = generate_wheel_parts()
    wheel_all_tris = np.concatenate([p['triangles'] for p in wheel_parts], axis=0)

    write_dae(f'{mesh_dir}/wheel.dae', wheel_parts)
    write_dae(f'{mesh_dir}/wheel_left.dae', wheel_parts)
    write_dae(f'{mesh_dir}/wheel_right.dae', wheel_parts)

    write_stl(f'{mesh_dir}/wheel.stl', wheel_all_tris)
    write_stl(f'{mesh_dir}/wheel_left.stl', wheel_all_tris)
    write_stl(f'{mesh_dir}/wheel_right.stl', wheel_all_tris)

    # 2. Steering Bracket Meshes (Outboard drive motor & encoder, U-fork, axle)
    steer_left_parts = generate_steer_bracket_parts(side='left')
    steer_right_parts = generate_steer_bracket_parts(side='right')

    steer_left_tris = np.concatenate([p['triangles'] for p in steer_left_parts], axis=0)
    steer_right_tris = np.concatenate([p['triangles'] for p in steer_right_parts], axis=0)

    write_dae(f'{mesh_dir}/steer_bracket.dae', steer_left_parts)
    write_dae(f'{mesh_dir}/fl_steer_bracket.dae', steer_left_parts)
    write_dae(f'{mesh_dir}/rl_steer_bracket.dae', steer_left_parts)
    write_dae(f'{mesh_dir}/fr_steer_bracket.dae', steer_right_parts)
    write_dae(f'{mesh_dir}/rr_steer_bracket.dae', steer_right_parts)

    write_stl(f'{mesh_dir}/steer_bracket.stl', steer_left_tris)
    write_stl(f'{mesh_dir}/fl_steer_bracket.stl', steer_left_tris)
    write_stl(f'{mesh_dir}/rl_steer_bracket.stl', steer_left_tris)
    write_stl(f'{mesh_dir}/fr_steer_bracket.stl', steer_right_tris)
    write_stl(f'{mesh_dir}/rr_steer_bracket.stl', steer_right_tris)

    # 3. Chassis Mesh
    chassis_parts = generate_chassis_parts()
    chassis_all_tris = np.concatenate([p['triangles'] for p in chassis_parts], axis=0)

    write_dae(f'{mesh_dir}/chassis.dae', chassis_parts)
    write_stl(f'{mesh_dir}/chassis.stl', chassis_all_tris)

    print('All Smart Car digital twin DAE and STL meshes generated successfully in SI METERS!')


if __name__ == '__main__':
    main()
