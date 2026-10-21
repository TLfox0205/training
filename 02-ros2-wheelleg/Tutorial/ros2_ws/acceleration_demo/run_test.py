from pathlib import Path
from datetime import datetime
import subprocess,os,time,signal,json,sys
ws=Path(__file__).resolve().parents[1]
headless='--check' in sys.argv
record=Path(sys.argv[sys.argv.index('--record-dir')+1]) if '--record-dir' in sys.argv else ws/'src/wheel_leg_controller/evaluation'/('acceleration_demo_'+datetime.now().strftime('%Y%m%d_%H%M%S'))
record.mkdir(parents=True,exist_ok=True)
env=os.environ.copy();env['ROS_DOMAIN_ID']='190';processes=[]
def start(args):
 p=subprocess.Popen(args,env=env,start_new_session=True);processes.append(p);return p
print('加减速实验：站稳 → 前进 0.2 → 停车 → 后退 -0.2 → 停车；仿真倍率 1.0。',flush=True)
print('本次数据目录：',record,flush=True)
try:
 start(['python',str(ws/'acceleration_demo/schedule.py')]);time.sleep(1)
 start(['ros2','run','wheel_leg_sim','wheel_leg_sim','--ros-args','-p','viewer:='+('false' if headless else 'true'),'-p','realtime:='+('false' if headless else 'true'),'-p','rtf:=1.0','-p','publish_clock:=true']);time.sleep(1)
 start(['ros2','run','wheel_leg_controller','wheel_leg_controller','--ros-args','-p','max_steps:=8000','-p','record_dir:='+str(record)])
 deadline=time.monotonic()+90
 while time.monotonic()<deadline:
  reports=list(record.glob('balance_*.json'))
  if reports:
   r=json.loads(reports[0].read_text())
   print('最后 5 秒平衡/腿长检查：',r['steady_window_pass'],flush=True)
   print('数据已保存：',record,flush=True)
   break
  if any(p.poll() is not None for p in processes):raise RuntimeError('进程提前退出，请检查报错')
  time.sleep(.25)
 else:raise RuntimeError('实验超时')
except KeyboardInterrupt:print('实验中止，完整结果尚未生成。',flush=True)
finally:
 for p in reversed(processes):
  if p.poll() is None:
   os.killpg(p.pid,signal.SIGINT)
   try:p.wait(timeout=5)
   except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGTERM);p.wait(timeout=3)
