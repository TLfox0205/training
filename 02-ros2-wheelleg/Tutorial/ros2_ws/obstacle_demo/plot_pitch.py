"""Plot an actual recorded run; no simulation or recording is started."""
from pathlib import Path
import csv
import sys
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ws = Path(__file__).resolve().parents[1]
folder = ws / 'src/wheel_leg_controller/evaluation'
if len(sys.argv) > 1:
    csv_path = Path(sys.argv[1]).expanduser().resolve()
else:
    files = list(folder.glob('obstacle_demo_*/balance_*.csv'))
    if not files:
        raise SystemExit('没有找到越障 CSV，请先完成一次演示。')
    csv_path = max(files, key=lambda p: p.stat().st_mtime)
with csv_path.open() as f:
    rows = list(csv.DictReader(f))
t = [float(row['sim_time_s']) for row in rows]
pitch = [float(row['pitch_deg']) for row in rows]
if not t or t[-1] - t[0] < 5:
    raise SystemExit('数据不足 5 秒，不能计算最后 5 秒指标。')
start = t[-1] - 5.0
steady = [(x, y) for x, y in zip(t, pitch) if x >= start - 1e-9]
peak = max(abs(y) for _, y in steady)
fig, axes = plt.subplots(2, 1, figsize=(10, 7), constrained_layout=True)
axes[0].plot(t, pitch, color='tab:blue', linewidth=1, label='Measured pitch')
axes[0].axvspan(start, t[-1], color='tab:green', alpha=.15, label='Final 5 s')
axes[0].set_title('Full run: body pitch')
axes[0].legend(loc='upper right')
axes[1].plot([x for x, _ in steady], [y for _, y in steady], color='tab:blue')
axes[1].set_title(f'Final 5 s: maximum absolute pitch = {peak:.3f} deg')
axes[1].set_xlim(start, t[-1])
axes[1].set_ylim(-max(1, peak * 1.3), max(1, peak * 1.3))
for ax in axes:
    ax.set_xlabel('Simulation time (s)')
    ax.set_ylabel('Pitch (deg)')
    ax.grid(True, alpha=.3)
    ax.axhline(0, color='gray', linewidth=.7)
for bound in [-10, 10]:
    axes[0].axhline(bound, color='tab:red', linestyle='--', linewidth=1)
image = csv_path.parent / 'pitch_curve.png'
fig.savefig(image, dpi=180)
plt.close(fig)
print('数据来源：', csv_path)
print('曲线已保存：', image)
print(f'最后 5 秒的 pitch 最大绝对偏差：{peak:.3f} 度')
print('注意：仿真时间与实际录像时长不是同一计时口径。')
