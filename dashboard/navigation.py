"""
navigation.py
GPS-denied navigation via complementary filter fusing:
    - Lucas-Kanade optical flow  (primary)
    - IMU dead reckoning         (fallback)
    - Barometric altitude        (Z-axis)

Mahony complementary filter suppresses gyroscope drift in attitude estimation.
"""

import cv2
import numpy as np
import math


class MahonyFilter:
    """
    Mahony complementary filter for attitude estimation.
    Fuses accelerometer and gyroscope data in quaternion space.

    q_t = q_{t-1} ⊗ [1, 0.5 * omega * dt]

    Reference: Mahony et al., IEEE Trans. Automatic Control, 2008.
    """

    def __init__(self, kp: float = 2.0, ki: float = 0.005):
        self.kp = kp          # proportional gain
        self.ki = ki          # integral gain
        self.q  = np.array([1.0, 0.0, 0.0, 0.0])   # quaternion [w, x, y, z]
        self.integral_fb = np.zeros(3)

    def update(self, accel: np.ndarray, gyro: np.ndarray, dt: float):
        """
        Update quaternion estimate.
        accel: [ax, ay, az] in m/s^2
        gyro:  [gx, gy, gz] in rad/s
        """
        q = self.q
        ax, ay, az = accel
        gx, gy, gz = gyro

        norm = math.sqrt(ax*ax + ay*ay + az*az)
        if norm < 1e-6:
            return
        ax, ay, az = ax/norm, ay/norm, az/norm

        # Estimated gravity direction from quaternion
        vx = 2*(q[1]*q[3] - q[0]*q[2])
        vy = 2*(q[0]*q[1] + q[2]*q[3])
        vz = q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3]

        # Error is cross product of estimated and measured gravity
        ex = ay*vz - az*vy
        ey = az*vx - ax*vz
        ez = ax*vy - ay*vx

        # Integral feedback
        self.integral_fb += np.array([ex, ey, ez]) * self.ki * dt
        gx += self.kp*ex + self.integral_fb[0]
        gy += self.kp*ey + self.integral_fb[1]
        gz += self.kp*ez + self.integral_fb[2]

        # Integrate quaternion
        gx *= 0.5 * dt
        gy *= 0.5 * dt
        gz *= 0.5 * dt
        qa, qb, qc = q[0], q[1], q[2]
        q[0] += (-qb*gx - qc*gy - q[3]*gz)
        q[1] += ( qa*gx + qc*gz - q[3]*gy)
        q[2] += ( qa*gy - qb*gz + q[3]*gx)
        q[3] += ( qa*gz + qb*gy - qc*gx)

        norm = math.sqrt(sum(x*x for x in q))
        self.q = q / norm

    @property
    def euler_angles(self) -> tuple[float, float, float]:
        """Returns (roll, pitch, yaw) in degrees."""
        q = self.q
        roll  = math.degrees(math.atan2(2*(q[0]*q[1]+q[2]*q[3]),
                                         1-2*(q[1]**2+q[2]**2)))
        pitch = math.degrees(math.asin(max(-1, min(1, 2*(q[0]*q[2]-q[3]*q[1])))))
        yaw   = math.degrees(math.atan2(2*(q[0]*q[3]+q[1]*q[2]),
                                         1-2*(q[2]**2+q[3]**2)))
        return roll, pitch, yaw


class NavigationFusion:
    """
    Complementary filter fusing optical flow, IMU, and barometric altitude.

    p_t = w1 * p_flow + w2 * p_IMU + w3 * p_pipe

    Weights reflect estimated reliability:
        optical flow  (w1 = 0.6)  -- primary, good in textured environments
        IMU           (w2 = 0.3)  -- fallback when flow is unreliable
        pipe align    (w3 = 0.1)  -- lateral correction anchor
    """

    W_FLOW = 0.6
    W_IMU  = 0.3
    W_PIPE = 0.1

    def __init__(self):
        self.mahony   = MahonyFilter()
        self.distance = 0.0          # total distance traveled (m)
        self.velocity = np.zeros(2)  # [vx, vy] in m/s
        self.prev_gray = None
        self.prev_pts  = None
        self.prev_time = None

        # LK optical flow params
        self.lk_params = dict(
            winSize=(21, 21),
            maxLevel=3,
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01)
        )
        self.feature_params = dict(
            maxCorners=200,
            qualityLevel=0.01,
            minDistance=7,
            blockSize=7
        )

    def update(self, frame: np.ndarray, imu: dict, altitude_m: float):
        """
        Update navigation estimate from new frame + IMU reading.
        imu: dict with keys AX, AY, AZ, GX, GY, GZ
        """
        import time
        now = time.monotonic()
        dt  = (now - self.prev_time) if self.prev_time else 0.1
        self.prev_time = now

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # ── Optical flow displacement ────────────────────────────────────
        flow_disp = 0.0
        if self.prev_gray is not None and self.prev_pts is not None:
            pts_new, status, _ = cv2.calcOpticalFlowPyrLK(
                self.prev_gray, gray, self.prev_pts, None, **self.lk_params)
            if pts_new is not None and status is not None:
                good_new = pts_new[status.flatten() == 1]
                good_old = self.prev_pts[status.flatten() == 1]
                if len(good_new) > 5:
                    disp = good_new - good_old
                    flow_disp = float(np.median(np.linalg.norm(disp, axis=1)))
                    # Convert pixels to meters (assumes ~0.5mm/pixel at 30cm height)
                    px_to_m = 0.0005
                    flow_disp *= px_to_m

        # ── IMU dead reckoning ───────────────────────────────────────────
        accel = np.array([imu.get("AX",0), imu.get("AY",0), imu.get("AZ",0)])
        gyro  = np.array([imu.get("GX",0), imu.get("GY",0), imu.get("GZ",0)])
        self.mahony.update(accel, gyro, dt)

        # Subtract gravity, integrate for velocity, then position
        g_world  = np.array([0, 0, 9.81])
        accel_ms = accel - g_world[:len(accel)]
        self.velocity += accel_ms[:2] * dt
        imu_disp = float(np.linalg.norm(self.velocity * dt))

        # ── Fused displacement ───────────────────────────────────────────
        fused_disp = (self.W_FLOW * flow_disp +
                      self.W_IMU  * imu_disp)
        self.distance += fused_disp

        # ── Refresh feature points ───────────────────────────────────────
        self.prev_gray = gray.copy()
        self.prev_pts  = cv2.goodFeaturesToTrack(
            gray, mask=None, **self.feature_params)
        if self.prev_pts is not None:
            self.prev_pts = self.prev_pts.reshape(-1, 1, 2).astype(np.float32)
