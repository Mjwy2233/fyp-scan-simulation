"""Single bounded ROS planning smoke test. No hardware commands are published."""
import argparse
import csv
import datetime
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import threading
import time


def run(kind,direction,limit):
    if os.environ.get('ROS_MASTER_URI') != 'http://127.0.0.1:11381':
        raise RuntimeError('Use the dedicated simulation master at http://127.0.0.1:11381; see README.')
    active=subprocess.check_output(['rosnode','list'],text=True,timeout=5).splitlines()
    if set(active) != {'/rosout'}:
        raise RuntimeError('Expected an idle simulation master with only /rosout.')
    import rospy
    from nav_msgs.msg import Odometry,Path as RosPath
    from geometry_msgs.msg import PoseStamped
    from sensor_msgs.msg import PointCloud2
    from scan_planner.msg import Bspline
    root=Path(__file__).resolve().parents[1]
    stamp=datetime.datetime.now().strftime('%Y%m%d-%H%M%S')
    out=root/'runs'/(stamp+'_'+kind+'_'+direction);out.mkdir(parents=True,exist_ok=False)
    manifest=json.loads((root/'configs/configuration_manifest.json').read_text())
    rows=list(csv.DictReader((root/'maps'/kind/'profile.csv').open()))
    if direction=='down':rows.reverse()
    target=[float(rows[-1][k]) for k in ('x_m','y_m','body_reference_z_m')]
    result={'scene':kind,'direction':direction,'status':'started','manifest':manifest,
            'target_body_xyz':target,'limit_s':limit,'arrival_tolerance_m':0.2,'hold_s':1.0,
            'physics_validated':False,'local_date':datetime.date.today().isoformat()}
    start=time.monotonic();proc=None;bag=None;f=None
    state={'odom':0,'cloud':0,'bspline':0,'last':None,'hold':None,'reached':False}
    arrival_lock=threading.Lock()
    sent=None
    try:
        log=(out/'roslaunch.log').open('w')
        proc=subprocess.Popen(['roslaunch',str(root/'configs'/(kind+'_'+direction+'.launch'))],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        rospy.init_node('fyp_pilot_recorder',anonymous=True,disable_signals=True)
        f=(out/'odometry.csv').open('w',newline='');w=csv.writer(f)
        w.writerow(['wall_elapsed_s','ros_stamp_s','x_m','y_m','z_m','vx','vy','vz','target_error_m'])
        def odom(msg):
            pos=msg.pose.pose.position;vel=msg.twist.twist.linear
            error=math.dist([pos.x,pos.y,pos.z],target)
            now=time.monotonic();state['odom']+=1;state['last']=[pos.x,pos.y,pos.z]
            w.writerow([now-start,msg.header.stamp.to_sec(),pos.x,pos.y,pos.z,vel.x,vel.y,vel.z,error])
            with arrival_lock:
                if sent is not None and now>=sent and error<=0.2:
                    if state['hold'] is None:state['hold']=now
                    if now-state['hold']>=1:state['reached']=True
                else:state['hold']=None
        def cloud(msg):state['cloud']+=int(msg.width*msg.height>0)
        def spline(msg):state['bspline']+=1
        subscriptions=[rospy.Subscriber('/quad_0/body_pose',Odometry,odom,queue_size=200),
                       rospy.Subscriber('/pcl_render_node/cloud',PointCloud2,cloud,queue_size=1),
                       rospy.Subscriber('/planning/bspline',Bspline,spline,queue_size=20)]
        pub=rospy.Publisher('/initial_path',RosPath,queue_size=1,latch=True)
        bag=subprocess.Popen(['rosbag','record','-O',str(out/'pilot.bag'),'/quad_0/body_pose','/planning/bspline','/initial_path'],stdout=(out/'rosbag.log').open('w'),stderr=subprocess.STDOUT,start_new_session=True)
        ready_deadline=time.monotonic()+20
        while time.monotonic()<ready_deadline:
            if proc.poll() is not None:raise RuntimeError('roslaunch exited during startup')
            if state['odom']>=10 and state['cloud']>=3 and pub.get_num_connections()>0:break
            time.sleep(0.1)
        else:raise RuntimeError('Timed out waiting for odometry, cloud, or path subscriber')
        subprocess.run(['rosparam','dump',str(out/'runtime_params.yaml')],timeout=10,check=True)
        path=RosPath();path.header.frame_id='world';path.header.stamp=rospy.Time.now()
        for row in rows:
            ps=PoseStamped();ps.header=path.header
            ps.pose.position.x=float(row['x_m']);ps.pose.position.y=float(row['y_m'])
            # This checkout adds body_height_ to every incoming path position.
            ps.pose.position.z=float(row['body_reference_z_m'])-manifest['body_height_added_by_path_callback']
            ps.pose.orientation.w=1;path.poses.append(ps)
        with arrival_lock:
            pub.publish(path);sent=time.monotonic()
        result['reference_sent_s']=sent-start
        while time.monotonic()-sent<limit:
            if proc.poll() is not None:raise RuntimeError('roslaunch exited during planning')
            if state['reached']:break
            time.sleep(.1)
        result.update(status='reached_in_open_loop' if state['reached'] else 'timeout',
                      elapsed_after_reference_s=time.monotonic()-sent)
    except Exception as exc:result.update(status='error',error=str(exc))
    finally:
        for child in (bag,proc):
            if child and child.poll() is None:
                os.killpg(child.pid,signal.SIGINT)
                try:child.wait(timeout=12)
                except subprocess.TimeoutExpired:
                    os.killpg(child.pid,signal.SIGTERM);child.wait(timeout=5)
        for subscription in locals().get('subscriptions', []):
            subscription.unregister()
        if f:f.close()
        result.update(state)
        result['hold_start_after_reference_s'] = (
            state['hold']-sent if state['hold'] is not None and sent is not None else None)
        del result['hold']
        (out/'result.json').write_text(json.dumps(result,indent=2))
        print(json.dumps(result,indent=2))
        print('Results:',out)
    return result['status']=='reached_in_open_loop'


if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('--scene',choices=['flat','ramp','stairs'],default='ramp')
    ap.add_argument('--direction',choices=['up','down'],default='up');ap.add_argument('--limit',type=float,default=60)
    a=ap.parse_args()
    if not math.isfinite(a.limit) or a.limit<=0:
        ap.error('--limit must be positive and finite')
    raise SystemExit(0 if run(a.scene,a.direction,a.limit) else 1)
