class LinearCvKalmanFilter:
    def __init__(self, process_noise_pos, process_noise_vel, measure_noise_pos):
        self._q_pos = float(process_noise_pos)
        self._q_vel = float(process_noise_vel)
        self._r_pos = float(measure_noise_pos)
        if self._q_pos < 0.0 or self._q_vel < 0.0 or self._r_pos <= 0.0:
            raise ValueError("invalid kalman noise parameters")
        self._x = [[0.0, 0.0] for _ in range(3)]
        self._p = [[[1.0, 0.0], [0.0, 1.0]] for _ in range(3)]

    def reset(self, p_xyz):
        for i in range(3):
            self._x[i][0] = float(p_xyz[i])
            self._x[i][1] = 0.0
            self._p[i] = [[1.0, 0.0], [0.0, 1.0]]

    def predict(self, dt):
        dt = float(dt)
        if dt < 0.0:
            raise ValueError("dt must be >= 0")
        dt2 = dt * dt
        for i in range(3):
            p00, p01 = self._p[i][0]
            p10, p11 = self._p[i][1]
            self._x[i][0] += dt * self._x[i][1]
            p00_new = p00 + dt * (p01 + p10) + dt2 * p11 + self._q_pos
            p01_new = p01 + dt * p11
            p10_new = p10 + dt * p11
            p11_new = p11 + self._q_vel
            self._p[i] = [[p00_new, p01_new], [p10_new, p11_new]]

    def update(self, z_xyz):
        for i in range(3):
            z = float(z_xyz[i])
            p00, p01 = self._p[i][0]
            p10, p11 = self._p[i][1]
            y = z - self._x[i][0]
            s = p00 + self._r_pos
            if s <= 1e-12:
                raise ValueError("invalid innovation covariance")
            k0 = p00 / s
            k1 = p10 / s
            self._x[i][0] += k0 * y
            self._x[i][1] += k1 * y
            self._p[i] = [
                [(1.0 - k0) * p00, (1.0 - k0) * p01],
                [p10 - k1 * p00, p11 - k1 * p01],
            ]

    def position(self):
        return [self._x[i][0] for i in range(3)]

    def velocity(self):
        return [self._x[i][1] for i in range(3)]
