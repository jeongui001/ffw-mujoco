"""Control-loop cadence measured from actual monotonic start timestamps."""


class LoopTiming:
    def __init__(self, alpha):
        if not 0 < alpha <= 1:
            raise ValueError("Frequency EMA alpha must be in (0, 1]")
        self.alpha = alpha
        self.reset()

    def reset(self):
        self.previous = None
        self.frequency = 0.0
        self.ema = 0.0

    def record(self, started):
        if self.previous is not None and started > self.previous:
            self.frequency = 1.0 / (started - self.previous)
            self.ema = (self.frequency if self.ema == 0 else
                        self.alpha * self.frequency + (1 - self.alpha) * self.ema)
        self.previous = started
