from pathlib import Path
from datetime import datetime
import subprocess,os,time,signal,json
ws=Path(__file__).resolve().parents[1]
record=ws/'src/wheel_leg_controller/evaluation'/('obstacle_demo_'+datetime.now().strftime('%Y%m%d_%H%M%S'))
record.mkdir(parents=True)
env=os.environ.copy();env['ROS_DOMAIN_ID']='188'
processes=[]
def start(args):
 p=subprocess.Popen(args,env=env,start_new_session=True);processes.append(p);return p
print('新建独立仿真：目标速度 0.625 m/s，仿真倍率 1.0（不加速），20 秒仿真时间对应约 20 秒实际运行。按 Ctrl+C 可随时结束。',flush=True)
print('CSV/JSON 数据目录：',record,flush=True)
try:
 start(['python',str(ws/'obstacle_demo/speed_schedule.py')]);time.sleep(1)
 start(['ros2','run','wheel_leg_sim','wheel_leg_sim','--ros-args','-p','viewer:=true','-p','realtime:=true','-p','rtf:=1.0','-p','publish_clock:=true']);time.sleep(1)
 start(['ros2','run','wheel_leg_controller','wheel_leg_controller','--ros-args','-p','target_length:=0.1','-p','max_steps:=10000','-p','record_dir:='+str(record)])
 deadline=time.monotonic()+180
 while time.monotonic()<deadline:
  reports=list(record.glob('balance_*.json'))
  if reports:
   r=json.loads(reports[0].read_text())
   print('演示结束；最后 5 秒 pitch 最大偏差：',round(r['steady_pitch_abs_max_deg'],3),'度。',flush=True)
   break
  if any(p.poll() is not None for p in processes):raise RuntimeError('进程提前退出，请查看上面的错误信息')
  time.sleep(.25)
 else:raise RuntimeError('等待超时，请查看控制器输出')
except KeyboardInterrupt:print('已停止演示。',flush=True)
finally:
 for p in reversed(processes):
  if p.poll() is None:
   os.killpg(p.pid,signal.SIGINT)
   try:p.wait(timeout=5)
   except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGTERM);p.wait(timeout=3)
