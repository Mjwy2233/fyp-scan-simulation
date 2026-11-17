"""Generate isolated launch files from the installed checkout; never edit it."""
import argparse
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET


def configure(project, root):
    launch=project/'src/planner/plan_manage/launch'
    source=project/'src/planner/plan_manage/src/scan_replan_fsm.cpp'
    if 'pose_stamped.pose.position.z + body_height_' not in source.read_text(encoding='utf-8'):
        raise RuntimeError('Reference-path z convention differs; inspect before configuring')
    adv=ET.parse(launch/'advanced_param.xml').getroot()
    body=float(adv.find(".//param[@name='grid_map/body_height']").get('value'))
    out=root/'configs';out.mkdir(exist_ok=True)
    for kind in ('flat','ramp','stairs'):
        truth=json.loads((root/'maps'/kind/'scene.json').read_text(encoding='utf-8'))
        for direction in ('up','down'):
            run=ET.parse(launch/'run.launch').getroot()
            for name,value in [('is_real_world','false'),('navi_mode','3'),('controller_mode','open_loop')]:
                a=run.find("arg[@name='%s']"%name)
                if a is None:raise RuntimeError('Missing launch argument '+name)
                a.attrib.pop('default',None);a.set('value',value)
            for inc in run.findall('include'):
                if inc.get('file').endswith('/advanced_param.xml'):
                    inc.set('file',str(launch/'advanced_param.xml'))
                elif inc.get('file').endswith('/simulator.xml'):
                    inc.set('file',str(launch/'simulator.xml'))
                    for name,value in {
                        'use_pcd_map':'true','pcd_map_file':str(root/'maps'/kind/'map.pcd'),
                        'init_x':'-4.0' if direction=='up' else '4.0','init_y':'0.0',
                        'init_z':str(0.6 if direction=='up' else truth['total_rise_m']+0.6),
                        'map_size_x':'14.0','map_size_y':'8.0','map_size_z':'5.0',
                        'downsample_resolution':'0.04','pcd_publish_rate':'1.0',
                        'use_gpu':'false'}.items():
                        ET.SubElement(inc,'arg',{'name':name,'value':value})
            ET.ElementTree(run).write(out/(kind+'_'+direction+'.launch'),encoding='utf-8',xml_declaration=True)
    info={'project':str(project),'body_height_added_by_path_callback':body,
          'source_sha256':{str(p.relative_to(project)):hashlib.sha256(p.read_bytes()).hexdigest()
                           for p in [launch/'run.launch',launch/'simulator.xml',launch/'advanced_param.xml',source]},
          'method':'current_modified_checkout_truth_assisted_reference',
          'robot_parameters':'existing Go2 defaults, NOT M20 calibrated',
          'controller':'open_loop','navigation_mode':3}
    (out/'configuration_manifest.json').write_text(json.dumps(info,indent=2))
    return info


if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('--project',type=Path,default=Path.home()/'SCAN-Planner')
    ap.add_argument('--root',type=Path,default=Path(__file__).resolve().parents[1]);a=ap.parse_args()
    print(json.dumps(configure(a.project.resolve(),a.root.resolve()),indent=2))
