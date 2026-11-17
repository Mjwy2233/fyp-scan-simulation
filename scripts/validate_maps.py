"""Independent checks of generated PCD counts and known terrain dimensions."""
import hashlib
import json
import math
from pathlib import Path

root=Path(__file__).resolve().parents[1]/'maps'
for kind in ('flat','ramp','stairs'):
    folder=root/kind;truth=json.loads((folder/'scene.json').read_text())
    data=(folder/'map.pcd').read_text().split('DATA ascii\n')
    points=[tuple(map(float,line.split())) for line in data[1].splitlines()]
    assert len(points)==truth['points']==int(data[0].split('POINTS ')[1].strip())
    assert len(set(points))==len(points)
    assert all(math.isfinite(v) for p in points for v in p)
    assert hashlib.sha256((folder/'map.pcd').read_bytes()).hexdigest()==truth['pcd_sha256']
    for axis,bounds in enumerate(truth['bounds']):
        assert abs(min(p[axis] for p in points)-bounds[0])<1e-6
        assert abs(max(p[axis] for p in points)-bounds[1])<1e-6
    if kind=='ramp':
        center=[p for p in points if -1.9<p[0]<1.9 and abs(p[1])<1e-6]
        assert len(center)>50
        assert all(abs(p[2]-(p[0]+2)*math.tan(math.radians(10)))<1e-6 for p in center)
    if kind=='stairs':
        for i in range(6):
            x=-0.9+i*0.3
            riser={round(p[2],6) for p in points if abs(p[0]-x)<1e-6 and abs(p[1])<1e-6}
            assert round(i*.12,6) in riser and round((i+1)*.12,6) in riser
            tread_x=x+.15
            # Nearest sampled surface at tread centre has expected support height.
            tread=[p for p in points if abs(p[0]-tread_x)<.025 and abs(p[1])<1e-6]
            assert tread and all(abs(p[2]-(i+1)*.12)<1e-6 for p in tread)
    print(kind,'PASS',len(points),'points')
