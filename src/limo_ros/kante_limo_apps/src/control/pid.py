class PID:
    def __init__(self, kp=0.2, ki=0.0, kd=0.01, i_max=None):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.i = 0.0
        self.prev_e = None
        self.i_max = i_max

    def compute(self, e, dt):
        self.i += e*dt
        if self.i_max is not None:
            self.i = max(min(self.i, self.i_max), -self.i_max)
        d = 0.0 if self.prev_e is None else (e - self.prev_e)/dt
        self.prev_e = e
        return self.kp*e + self.ki*self.i + self.kd*d