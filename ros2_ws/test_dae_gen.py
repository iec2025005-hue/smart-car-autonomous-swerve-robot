#!/usr/bin/env python3
"""
Test Collada DAE generation for Smart Car swerve robot.
"""

import struct
import numpy as np
import os

def format_floats(arr):
    return ' '.join(f'{x:.4f}' for x in arr.flatten())

def format_ints(arr):
    return ' '.join(str(x) for x in arr.flatten())

def write_dae(filename, parts):
    """
    Write multi-part, multi-material COLLADA 1.4.1 (.dae) mesh.
    parts: list of dicts:
      - 'id': str
      - 'name': str
      - 'color': (r, g, b, a) float
      - 'triangles': (N, 3, 3) np.ndarray
    """
    out = ['<?xml version="1.0" encoding="utf-8"?>']
    out.append('<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">')
    out.append('  <asset>')
    out.append('    <contributor><authoring_tool>SmartCar CAD Digital Twin</authoring_tool></contributor>')
    out.append('    <created>2026-09-01T00:00:00</created>')
    out.append('    <modified>2026-09-01T00:00:00</modified>')
    out.append('    <unit name="millimeter" meter="0.001"/>')
    out.append('    <up_axis>Z_UP</up_axis>')
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
        out.append(f'            <ambient><color sid="ambient">{r*0.3:.3f} {g*0.3:.3f} {b*0.3:.3f} 1</color></ambient>')
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

        # Flatten vertices (3 per triangle)
        verts = tris.reshape(-1, 3)
        # Compute face normals
        norms = np.zeros_like(verts)
        for i in range(num_tri):
            v0, v1, v2 = tris[i]
            n = np.cross(v1 - v0, v2 - v0)
            nl = np.linalg.norm(n)
            n = n / nl if nl > 1e-9 else np.array([0.0, 0.0, 1.0])
            norms[i*3] = n
            norms[i*3 + 1] = n
            norms[i*3 + 2] = n

        pos_str = format_floats(verts)
        norm_str = format_floats(norms)
        # Index pairs (vert_idx, norm_idx)
        indices = np.arange(len(verts))
        p_pairs = np.column_stack((indices, indices)).flatten()
        p_str = format_ints(p_pairs)

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

    content = '\n'.join(out)
    with open(filename, 'w', encoding='utf-8') as f:
        f.write(content)
    print(f"Generated {filename}: {len(parts)} parts, {sum(len(p['triangles']) for p in parts)} total triangles")

