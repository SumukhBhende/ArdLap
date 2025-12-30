# jog_controller.py
import math
import time

class JogController:
    """
    Converts velocity commands into GRBL jog commands ($J=)
    """

    def __init__(self,
                 max_feed=1200,     # mm/min
                 min_feed=100,
                 jog_distance=10000,
                 accel_limit=3000   # mm/min^2
                 ):
        self.max_feed = max_feed
        self.min_feed = min_feed
        self.jog_distance = jog_distance
        self.accel_limit = accel_limit
        self.is_jogging = False
        self.last_feed = 0
        self.last_time = time.time()

    def _limit_accel(self, target_feed):
        now = time.time()
        dt = now - self.last_time
        self.last_time = now

        max_delta = self.accel_limit * dt
        delta = target_feed - self.last_feed

        if abs(delta) > max_delta:
            target_feed = self.last_feed + max_delta * (1 if delta > 0 else -1)

        self.last_feed = target_feed
        return target_feed

    def velocity_to_jog(self, vel_x, vel_y, deadband=0.001):
        """
        vel_x, vel_y: signed velocity values from PD controller
        """

        if abs(vel_x) < deadband and abs(vel_y) < deadband:
            return None

        # Direction only
        x = 0
        y = 0

        if abs(vel_x) >= deadband:
            x = self.jog_distance if vel_x > 0 else -self.jog_distance

        if abs(vel_y) >= deadband:
            y = self.jog_distance if vel_y > 0 else -self.jog_distance

        # Speed magnitude → feed rate
        speed = math.sqrt(vel_x**2 + vel_y**2)

        feed = speed * 1000  # SCALE FACTOR (tune if needed)
        feed = max(self.min_feed, min(feed, self.max_feed))
        feed = self._limit_accel(feed)
        self.is_jogging = True
        return f"$J=G91 X{x} Y{y} F{int(feed)}"


