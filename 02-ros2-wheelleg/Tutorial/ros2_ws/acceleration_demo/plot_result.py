from pathlib import Path
import csv,json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ws=Path(__file__).resolve().parents[1]
folders=list((ws/'src/wheel_leg_controller/evaluation').glob('acceleration_demo_*'))
if not folders:raise SystemExit('未找到加减速实验数据。')
folder=max(folders,key=lambda p:p.stat().st_mtime)
files=list(folder.glob('balance_*.csv'))
if len(files)!=1:raise SystemExit('CSV 文件数量异常，请检查数据目录。')
with files[0].open() as f:rows=list(csv.DictReader(f))
values=lambda key:np.array([float(r[key]) for r in rows])
t=values('sim_time_s');target=values('target_speed_m_s');ramp=values('ramped_speed_m_s');actual=values('wheel_speed_m_s')
if len(t)!=8001 or abs(t[-1]-16)>1e-9:raise SystemExit('记录步数或结束时间异常，暂不判定通过。')
dt=np.diff(t)
if np.any(dt<=0):raise SystemExit('时间序列不严格递增。')
a=np.diff(ramp)/dt;measured=np.diff(actual)/dt
moving=np.abs(np.diff(ramp))>1e-10
up=moving & (np.abs(ramp[1:])>np.abs(ramp[:-1])+1e-10)
down=moving & ~up
max_up=float(np.max(np.abs(a[up])));max_down=float(np.max(np.abs(a[down])))
valid=bool(max_up<=.8+1e-8 and max_down<=1.4+1e-8)
fig,axes=plt.subplots(2,1,figsize=(11,7),constrained_layout=True)
axes[0].step(t,target,where='post',linestyle='--',color='gray',label='Raw target /cmd_vel')
axes[0].plot(t,ramp,color='tab:orange',label='Ramped reference')
axes[0].plot(t,actual,color='tab:blue',linewidth=1,label='Measured wheel-rim speed')
axes[0].set_ylabel('Speed (m/s)');axes[0].set_title('Acceleration limiting: target, reference, and wheel response')
axes[0].legend(loc='best')
axes[1].plot(t[1:],a,color='tab:orange',linewidth=1,label='Rate of ramped reference')
for limit in [-1.4,-.8,.8,1.4]:axes[1].axhline(limit,color='gray',linestyle='--',linewidth=.7)
axes[1].set_ylabel('Reference rate (m/s^2)');axes[1].set_title(f'Reference limits: acceleration {max_up:.3f}, braking {max_down:.3f} m/s^2')
axes[1].legend(loc='best')
for ax in axes:ax.set_xlim(4.8,9);ax.set_xlabel('Simulation time (s)');ax.grid(alpha=.3)
image=folder/'acceleration_curve.png';fig.savefig(image,dpi=180);plt.close(fig)
report={'csv_file':str(files[0]),'row_count':len(rows),'sim_duration_s':float(t[-1]),'ramp_acceleration_max_m_s2':max_up,'ramp_braking_max_m_s2':max_down,'reference_slew_limits_pass':valid,'measured_wheel_acceleration_abs_max_5_to_9s_m_s2':float(np.max(np.abs(measured[(t[1:]>=5)&(t[1:]<=9)]))),'note':'Limits apply to the internal speed reference, not a hard bound on physical wheel/body acceleration.'}
(folder/'acceleration_check.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))
print('曲线已保存：',image)
print(f'内部目标：最大加速斜率 {max_up:.3f} m/s²，最大减速斜率 {max_down:.3f} m/s²')
print('目标速度斜坡限制检查：',valid)
print(f"实测轮缘速度差分加速度峰值（5–9秒）：{report['measured_wheel_acceleration_abs_max_5_to_9s_m_s2']:.3f} m/s²；它不受目标斜坡的同等硬限幅。")
