"""Deterministic surface PCDs and analytic truth; Python standard library only."""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path


def axis(a, b, spacing):
    n = max(1, math.ceil((b-a)/spacing))
    return [a+(b-a)*i/n for i in range(n+1)]


def height(x, kind):
    if kind == 'flat': return 0.0
    if kind == 'ramp': return min(4.0, max(0.0, x+2))*math.tan(math.radians(10))
    return min(6, max(0, math.floor((x+0.9)/0.3+1e-8)+1))*0.12


def generate(root, kind):
    out = root / kind
    out.mkdir(parents=True, exist_ok=True)
    spacing = 0.04
    pts = set()
    def add(x,y,z): pts.add(tuple(round(v,6) for v in (x,y,z)))
    ys = axis(-2,2,spacing)
    # Horizontal/slope support surfaces with explicit tread edges.
    edges = [-5,-2,2,5] if kind != 'stairs' else [-5,-0.9,-0.6,-0.3,0,0.3,0.6,0.9,5]
    for a,b in zip(edges,edges[1:]):
        zmid = height((a+b)/2,kind)
        for x in axis(a,b,spacing):
            z = zmid if kind == 'stairs' else height(x,kind)
            for y in ys: add(x,y,z)
    # Include all stair risers, side faces and end caps, not just tread points.
    if kind == 'stairs':
        for i in range(6):
            x = -0.9+i*0.3
            for z in axis(i*0.12,(i+1)*0.12,spacing):
                for y in ys: add(x,y,z)
    for x in axis(-5,5,spacing):
        for z in axis(-0.12,height(x,kind),spacing):
            for y in (-2,2): add(x,y,z)
    for x in (-5,5):
        for z in axis(-0.12,height(x,kind),spacing):
            for y in ys: add(x,y,z)
    pts = sorted(pts)
    pcd = out/'map.pcd'
    header = '# .PCD v0.7\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH %d\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS %d\nDATA ascii\n' % (len(pts),len(pts))
    pcd.write_text(header+''.join('%.6f %.6f %.6f\n'%v for v in pts),encoding='ascii')
    truth = dict(scene_id=kind,frame_id='world',units='m',bounds=[[-5,5],[-2,2],[-0.12,height(5,kind)]],
                 sample_spacing_max_m=spacing,points=len(pts),body_clearance_m=0.6,
                 body_clearance_status='debug reference only; not M20 calibrated',
                 slope_deg=10 if kind=='ramp' else 0 if kind=='flat' else None,
                 step_count=6 if kind=='stairs' else 0,step_height_m=0.12 if kind=='stairs' else None,
                 tread_depth_m=0.3 if kind=='stairs' else None,
                 total_rise_m=height(5,kind),pcd_sha256=hashlib.sha256(pcd.read_bytes()).hexdigest(),
                 truth_assisted_reference=True,physics_validated=False)
    (out/'scene.json').write_text(json.dumps(truth,indent=2),encoding='utf-8')
    with (out/'profile.csv').open('w',newline='') as f:
        w=csv.writer(f);w.writerow(['x_m','y_m','surface_z_m','body_reference_z_m'])
        for x in axis(-4,4,0.2):w.writerow([round(x,6),0,round(height(x,kind),6),round(height(x,kind)+0.6,6)])
    return truth


if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('--output',type=Path,default=Path(__file__).resolve().parents[1]/'maps')
    args=ap.parse_args()
    for kind in ('flat','ramp','stairs'):print(json.dumps(generate(args.output,kind)))
