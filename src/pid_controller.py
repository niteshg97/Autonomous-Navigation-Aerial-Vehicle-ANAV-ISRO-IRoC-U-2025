#A general PID controller class used for X and Y corrections (and can be reused for altitude).

# src/pid_controller.py
import time

class PID:
    def __init__(self, kp, ki, kd, setpoint=0.0, out_min=None, out_max=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        self._int = 0.0
        self._last_err = None
        self._last_time = None
        self.out_min = out_min
        self.out_max = out_max

    def reset(self):
        self._int = 0.0
        self._last_err = None
        self._last_time = None

    def compute(self, measurement, now=None):
        if now is None:
            now = time.time()
        err = self.setpoint - measurement
        if self._last_time is None:
            dt = 0.0
            derivative = 0.0
        else:
            dt = now - self._last_time
            derivative = (err - self._last_err) / dt if dt > 0 else 0.0

        self._int += err * dt
        out = self.kp * err + self.ki * self._int + self.kd * derivative

        # clamp
        if self.out_min is not None:
            out = max(self.out_min, out)
        if self.out_max is not None:
            out = min(self.out_max, out)

        self._last_err = err
        self._last_time = now
        return out
