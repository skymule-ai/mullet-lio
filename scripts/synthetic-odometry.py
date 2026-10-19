"""FAST-LIO odometry check without a sensor: a synthetic MID-360 (CustomMsg, IMU in g) in a box room,
replayed in real time. Still 0-2 s, accelerates in world x 2-3 s, then 0.5 m/s in world x while
yawing 0.3 rad/s. The MID-360 can be mounted tilted (roll, pitch in degrees on the command line).
Checks what images/fast-lio/patches/ change: the world is gravity-aligned (no height drift, body
"up" right) whatever the mount; the twist is in the body frame; the covariances are filled.
Run by scripts/smoke-fast-lio.sh, level and tilted.

    synthetic-odometry.py [roll_deg pitch_deg]"""
import sys
import time

import numpy as np
import rclpy
from builtin_interfaces.msg import Time
from livox_ros_driver2.msg import CustomMsg, CustomPoint
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

G = 9.81
T_LI = np.array([-0.011, -0.02329, 0.04412])
LO, HI = np.array([-10.0, -8.0, -2.0]), np.array([12.0, 9.0, 4.0])
DUR = 14.0


def pos(t):
    if t < 2: return np.zeros(3)
    if t < 3: return np.array([0.25 * (t - 2) ** 2, 0, 0])
    return np.array([0.25 + 0.5 * (t - 3), 0, 0])


def acc_w(t):
    return np.array([0.5, 0, 0]) if 2 <= t < 3 else np.zeros(3)


def yaw(t):
    return 0.3 * (t - 3) if t > 3 else 0.0


def rot(y):
    c, s = np.cos(y), np.sin(y)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def rot_xy(roll, pitch):
    cr, sr, cp, sp = np.cos(roll), np.sin(roll), np.cos(pitch), np.sin(pitch)
    return np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]]) @ np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])


R_M = rot_xy(*np.radians([float(a) for a in sys.argv[1:3]] or [0.0, 0.0]))  # body in a level, yawed frame


def stamp(t_abs):
    ns = int(round(t_abs * 1e9))
    return Time(sec=ns // 1_000_000_000, nanosec=ns % 1_000_000_000), ns


def main():
    rclpy.init()
    node = rclpy.create_node('synth_mid360')
    pub_imu = node.create_publisher(Imu, '/livox/imu', 50)
    pub_lid = node.create_publisher(CustomMsg, '/livox/lidar', 5)
    odoms = []
    node.create_subscription(Odometry, '/uav0/fast_lio/odometry',
                             lambda m: odoms.append(m), qos_profile_sensor_data)
    rng = np.random.default_rng(0)
    t0 = time.time() + 2.0  # let discovery settle
    k_imu, k_scan = 0, 0
    while True:
        t = time.time() - t0
        if t > DUR: break
        while k_imu * 0.005 <= t:
            ti = k_imu * 0.005
            if ti >= 0:
                R = rot(yaw(ti)) @ R_M
                f = R.T @ (acc_w(ti) + [0, 0, G]) / G
                w = R_M.T @ [0, 0, 0.3 if ti > 3 else 0.0]
                m = Imu()
                m.header.stamp, _ = stamp(t0 + ti)
                m.header.frame_id = 'livox_frame'
                m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z = f
                m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z = w
                pub_imu.publish(m)
            k_imu += 1
        if (k_scan + 1) * 0.1 <= t:
            ts = k_scan * 0.1
            n = 6000
            off = np.sort(rng.uniform(0, 0.1, n))
            az = rng.uniform(-np.pi, np.pi, n)
            el = np.radians(rng.uniform(-7, 52, n))
            d_l = np.stack([np.cos(el) * np.cos(az), np.cos(el) * np.sin(az), np.sin(el)], 1)
            msg = CustomMsg()
            msg.header.stamp, base_ns = stamp(t0 + ts)
            msg.header.frame_id = 'livox_frame'
            msg.timebase = base_ns
            tp = ts + off
            ys = np.array([yaw(x) for x in tp])
            ps = np.array([pos(x) for x in tp])
            c, sn = np.cos(ys), np.sin(ys)
            rotz = lambda v: np.stack([c * v[:, 0] - sn * v[:, 1], sn * v[:, 0] + c * v[:, 1], v[:, 2]], 1)
            o = ps + rotz(np.tile(R_M @ T_LI, (n, 1)))
            d = rotz(d_l @ R_M.T)
            with np.errstate(divide='ignore', invalid='ignore'):
                tt = np.where(d > 0, (HI - o) / d, (LO - o) / d)
            tt[~np.isfinite(tt) | (tt <= 0)] = np.inf
            p_l = d_l * tt.min(1)[:, None]
            pts = [CustomPoint(offset_time=int(off[i] * 1e9), x=float(p_l[i, 0]), y=float(p_l[i, 1]),
                               z=float(p_l[i, 2]), reflectivity=100, tag=0, line=i % 4) for i in range(n)]
            msg.points = pts
            msg.point_num = n
            pub_lid.publish(msg)
            k_scan += 1
        rclpy.spin_once(node, timeout_sec=0.001)
    end = time.time() + 1.0
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.05)

    assert len(odoms) > 50, f'only {len(odoms)} odometry messages'
    m = odoms[-1]
    t = (m.header.stamp.sec + m.header.stamp.nanosec * 1e-9) - t0
    p = m.pose.pose.position
    v = m.twist.twist.linear
    w = m.twist.twist.angular
    R_true = rot(yaw(t)) @ R_M
    v_exp = R_true.T @ [0.5, 0, 0]
    w_exp = R_M.T @ [0, 0, 0.3]
    q = m.pose.pose.orientation
    x, y, z, qw = q.x, q.y, q.z, q.w
    up_fl = np.array([2 * (x * z - qw * y), 2 * (y * z + qw * x), 1 - 2 * (x * x + y * y)])  # R^T e_z
    pc = np.array(m.pose.covariance).reshape(6, 6)
    tc = np.array(m.twist.covariance).reshape(6, 6)
    print(f'{len(odoms)} odometry msgs; last at t={t:.2f}')
    print(f'pos      {p.x:.3f} {p.y:.3f} {p.z:.3f}   expected horizontal {pos(t)[0]:.3f}, z 0')
    print(f'up_body  {np.round(up_fl, 3)}   expected {np.round(R_true.T[:, 2], 3)}')
    print(f'v_body   {v.x:.3f} {v.y:.3f} {v.z:.3f}   expected {np.round(v_exp, 3)}')
    print(f'w_body   {w.x:.3f} {w.y:.3f} {w.z:.3f}   expected {np.round(w_exp, 3)}')
    print('pose cov diag', np.diag(pc))
    print('twist cov diag', np.diag(tc))
    # The world's heading is the IMU's at start-up, so compare what does not depend on it.
    assert abs(np.hypot(p.x, p.y) - pos(t)[0]) < 0.2 and abs(p.z) < 0.1
    assert np.linalg.norm(up_fl - R_true.T[:, 2]) < 0.02
    assert np.linalg.norm(np.array([v.x, v.y, v.z]) - v_exp) < 0.1
    assert np.linalg.norm(np.array([w.x, w.y, w.z]) - w_exp) < 0.02
    assert (np.diag(pc) > 0).all() and (np.diag(tc) > 0).all()
    print('SYNTHETIC CHECK PASSED')


main()
