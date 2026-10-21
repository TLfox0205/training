"""Independent ROS2 controller; state input and actuator output use topics only."""
import csv
import hashlib
import json
import math
import time
from collections import OrderedDict
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, JointState
from rm_msgs.msg import WheelLegCmd
from .control_math import BalanceCore

JOINTS = ('right1', 'right2', 'left1', 'left2', 'rightwheel', 'leftwheel')
ACTUATORS = ('right_phi1', 'right_phi4', 'left_phi1', 'left_phi4',
             'right_wheel', 'left_wheel')


class WheelLegController(Node):
    def __init__(self):
        super().__init__('wheel_leg_controller')
        self.declare_parameter('control_mode', 'balance')
        self.declare_parameter('target_length', 0.1)
        self.declare_parameter('max_steps', 5000)
        self.declare_parameter('record_dir', str(
            Path(__file__).resolve().parents[1] / 'evaluation'))
        self.mode = self.get_parameter('control_mode').value
        self.target_length = float(self.get_parameter('target_length').value)
        self.max_steps = int(self.get_parameter('max_steps').value)
        if self.mode not in ('balance', 'vmc'):
            raise ValueError('control_mode must be balance or vmc')
        if not 0.09 <= self.target_length <= 0.17:
            raise ValueError('target_length must be between 0.09 and 0.17 m')
        if self.max_steps < 2501:
            raise ValueError('max_steps must exceed 2500 for a 5 s steady window')
        self.core = BalanceCore(self.target_length, self.mode)
        self.imu_buffer = OrderedDict()
        self.joint_buffer = OrderedDict()
        self.last_stamp = None
        self.steps = 0
        self.target_v = 0.0
        self.finished = False
        self.samples = []
        self.started_mono = None
        self.last_metrics = None
        folder = Path(self.get_parameter('record_dir').value).expanduser()
        folder.mkdir(parents=True, exist_ok=True)
        tag = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        self.csv_path = folder / f'{self.mode}_{tag}.csv'
        self.json_path = folder / f'{self.mode}_{tag}.json'
        self.csv_file = self.csv_path.open('w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            'step', 'sim_time_s', 'wall_time_s', 'message_stamp_s', 'pitch_deg',
            'right_length_m', 'left_length_m', 'wheel_position_m', 'wheel_speed_m_s',
            'target_speed_m_s', 'ramped_speed_m_s'])
        self.cmd_pub = self.create_publisher(WheelLegCmd, '/actuator_cmd', 10)
        self.create_subscription(Imu, '/imu', self.on_imu, 10)
        self.create_subscription(JointState, '/joint_states', self.on_joints, 10)
        self.create_subscription(Twist, '/cmd_vel', self.on_target, 10)
        self.create_timer(1.0, self.report)
        self.get_logger().info(
            f'等待成对状态；mode={self.mode}, 目标腿长={self.target_length:.3f} m, '
            f'最多 {self.max_steps} 步；请从刚重启的仿真器开始')

    @staticmethod
    def stamp(msg):
        return (msg.header.stamp.sec, msg.header.stamp.nanosec)

    def store(self, buffer, key, msg):
        buffer[key] = msg
        while len(buffer) > 20:
            buffer.popitem(last=False)

    def on_imu(self, msg):
        key = self.stamp(msg)
        self.store(self.imu_buffer, key, msg)
        self.process(key)

    def on_joints(self, msg):
        key = self.stamp(msg)
        self.store(self.joint_buffer, key, msg)
        self.process(key)

    def on_target(self, msg):
        if self.finished:
            return
        if math.isfinite(msg.linear.x):
            self.target_v = float(msg.linear.x)
        if abs(msg.angular.z) > 1e-9:
            self.get_logger().warn('本版未实现转向跟随，angular.z 不参与控制')

    def process(self, key):
        if self.finished or key not in self.imu_buffer or key not in self.joint_buffer:
            return
        if self.last_stamp is not None and key <= self.last_stamp:
            return
        imu = self.imu_buffer.pop(key)
        joints = self.joint_buffer.pop(key)
        positions = dict(zip(joints.name, joints.position))
        velocities = dict(zip(joints.name, joints.velocity))
        if not all(name in positions and name in velocities for name in JOINTS):
            return
        self.last_stamp = key
        q = imu.orientation
        # Reference coordinate convention: negate ROS quaternion-derived pitch and gyro Y.
        pitch = -math.asin(max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x))))
        gyro_pitch = -float(imu.angular_velocity.y)
        values = list(positions.values()) + list(velocities.values()) + [pitch, gyro_pitch]
        if not all(math.isfinite(value) for value in values):
            self.fail('状态包含非有限值')
            return
        try:
            torques, metrics = self.core.update(
                positions, velocities, pitch, gyro_pitch, self.target_v)
        except (ValueError, ZeroDivisionError, OverflowError) as exc:
            self.fail(str(exc))
            return
        if self.started_mono is None:
            self.started_mono = time.monotonic()
        wall_time = time.monotonic() - self.started_mono
        sim_time = self.steps * self.core.dt
        row = {'step': self.steps, 'sim_time_s': sim_time,
               'wall_time_s': wall_time, **metrics}
        self.samples.append(row)
        self.last_metrics = metrics
        self.csv_writer.writerow([
            self.steps, sim_time, wall_time, key[0] + key[1] * 1e-9,
            metrics['pitch_deg'], metrics['right_length_m'], metrics['left_length_m'],
            metrics['wheel_position_m'], metrics['wheel_speed_m_s'],
            self.target_v, self.core.current_v])
        if self.steps >= self.max_steps:
            self.finish()
            return
        command = WheelLegCmd()
        for name, value in zip(ACTUATORS, torques):
            setattr(command, name, value)
        self.cmd_pub.publish(command)
        self.steps += 1
        if self.steps % 500 == 0:
            self.csv_file.flush()

    def report(self):
        if self.finished:
            return
        if self.last_metrics is None:
            self.get_logger().info('等待 /imu 与 /joint_states 的同时间戳状态')
            return
        m = self.last_metrics
        self.get_logger().info(
            f'步数={self.steps}, 仿真时间≈{self.steps * self.core.dt:.2f}s, '
            f'pitch={m["pitch_deg"]:.3f} deg, '
            f'右腿={m["right_length_m"]:.5f}m, 左腿={m["left_length_m"]:.5f}m')

    def finish(self):
        self.finished = True
        self.cmd_pub.publish(WheelLegCmd())
        self.csv_file.flush()
        self.csv_file.close()
        start = (self.max_steps - 2500) * self.core.dt
        tail = [row for row in self.samples if row['sim_time_s'] >= start]
        pitch_max = max(abs(row['pitch_deg']) for row in tail)
        right_max = max(abs(row['right_length_m'] - self.target_length) for row in tail)
        left_max = max(abs(row['left_length_m'] - self.target_length) for row in tail)
        duration = tail[-1]['sim_time_s'] - tail[0]['sim_time_s']
        wall_duration = tail[-1]['wall_time_s'] - tail[0]['wall_time_s']
        hashes = {}
        for filename in ('vmc_cal.py', 'lqr_update.py'):
            hashes[filename] = hashlib.sha256(
                Path(__file__).with_name(filename).read_bytes()).hexdigest()
        summary = {
            'mode': self.mode, 'target_length_m': self.target_length,
            'dt_s': self.core.dt, 'command_steps': self.steps,
            'sim_duration_s': self.steps * self.core.dt,
            'wall_duration_s': self.samples[-1]['wall_time_s'],
            'steady_window_sim_s': duration, 'steady_window_wall_s': wall_duration,
            'steady_pitch_abs_max_deg': pitch_max,
            'steady_right_length_error_max_mm': right_max * 1000,
            'steady_left_length_error_max_mm': left_max * 1000,
            'steady_window_pass': bool(duration >= 5.0 - 1e-9 and pitch_max < 10.0
                                       and right_max <= 0.005 and left_max <= 0.005),
            'csv_file': str(self.csv_path), 'reference_sha256': hashes,
            'time_note': 'sim_time is command_steps * 0.002; start from a fresh sim',
            'motion_bonus_validated': False,
        }
        self.json_path.write_text(json.dumps(summary, ensure_ascii=False, indent=2))
        self.get_logger().info(
            f'测试完成：最后 {duration:.2f}s 仿真时间，'
            f'pitch最大偏差={pitch_max:.3f} deg，'
            f'腿长最大误差：右={right_max * 1000:.3f}mm、左={left_max * 1000:.3f}mm，'
            f'达标={summary["steady_window_pass"]}')
        self.get_logger().info(f'记录已保存：{self.json_path}')

    def fail(self, reason):
        self.finished = True
        self.cmd_pub.publish(WheelLegCmd())
        self.csv_file.flush()
        self.get_logger().error(f'控制已停止，未通过验收：{reason}')

    def close_record(self):
        if not self.csv_file.closed:
            self.csv_file.close()


def main(args=None):
    rclpy.init(args=args)
    node = WheelLegController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close_record()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
