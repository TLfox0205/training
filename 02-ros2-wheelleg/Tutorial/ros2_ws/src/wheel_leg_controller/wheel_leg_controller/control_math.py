"""ROS-independent control math, adapted from the tutorial reference algorithms."""
import math
from .vmc_cal import five_links_param
from .lqr_update import lqr_controller


def clip(value, low, high):
    return max(low, min(high, value))


class BalanceCore:
    def __init__(self, target_length=0.1, mode='balance'):
        self.dt = 0.002
        self.radius = 0.06
        self.target_length = target_length
        self.mode = mode
        self.legs = [five_links_param(self.dt), five_links_param(self.dt)]
        self.lqrs = [lqr_controller(self.dt), lqr_controller(self.dt)]
        self.previous_length_error = [0.0, 0.0]
        self.current_v = 0.0

    def update(self, positions, velocities, pitch, gyro_pitch, target_v=0.0):
        common_pos = (positions['leftwheel'] - positions['rightwheel']) / 2.0
        common_speed = (velocities['leftwheel'] - velocities['rightwheel']) / 2.0
        rate = 0.8 if abs(target_v) > abs(self.current_v) else 1.4
        difference = target_v - self.current_v
        self.current_v += clip(difference, -rate * self.dt, rate * self.dt)
        if abs(target_v - self.current_v) < 1e-12:
            self.current_v = target_v
        leg_torques, wheel_torques = [], []
        for side, leg, lqr in zip(('right', 'left'), self.legs, self.lqrs):
            flag = 0 if side == 'right' else 1
            leg.forward_kinematics_cal(
                flag, positions[side + '1'], positions[side + '2'], pitch, gyro_pitch)
            error = self.target_length - leg.L_0
            # Preserve the reference discrete PID: derivative is a per-step difference.
            radial_control = clip(
                700.0 * error + 5.0 * (error - self.previous_length_error[flag]),
                -150.0, 150.0)
            self.previous_length_error[flag] = error
            cosine = math.cos(leg.theta)
            feedforward = 4.0 / cosine if cosine != 0.0 else 0.0
            radial_force = clip(feedforward + radial_control, -100.0, 100.0)
            hip_torque, wheel_torque = 0.0, 0.0
            if self.mode == 'balance':
                direction = -1.0 if flag == 0 else 1.0
                lqr.states_update(
                    flag, leg, pitch, gyro_pitch,
                    direction * common_pos, direction * common_speed, self.radius)
                lqr.calc_k_matrix_from_poly(leg.L_0)
                lqr.v_set = self.current_v
                # Freeze the position target at rest, as required by the tutorial.
                if not (self.current_v == 0.0 and target_v == 0.0):
                    lqr.x_b_set = common_pos * self.radius
                state = (lqr.theta, lqr.d_theta, lqr.x_b - lqr.x_b_set,
                         lqr.v_b_whole - lqr.v_set, lqr.phi, lqr.d_phi)
                wheel_torque = sum(lqr.K[2 * i] * state[i] for i in range(6))
                hip_torque = sum(lqr.K[2 * i + 1] * state[i] for i in range(6))
                wheel_torque = clip(wheel_torque, -1.5, 1.5)
                if flag == 1:
                    wheel_torque = -wheel_torque
            t1, t4 = leg.VMC_torque_cal(radial_force, hip_torque)
            direction = 1.0 if flag == 0 else -1.0
            leg_torques.extend((direction * float(t1), direction * float(t4)))
            wheel_torques.append(float(wheel_torque))
        torques = leg_torques + wheel_torques
        if not all(math.isfinite(value) for value in torques):
            raise ValueError('non-finite actuator torque')
        return torques, {
            'pitch_deg': math.degrees(pitch),
            'right_length_m': self.legs[0].L_0,
            'left_length_m': self.legs[1].L_0,
            'wheel_position_m': common_pos * self.radius,
            'wheel_speed_m_s': common_speed * self.radius,
        }
