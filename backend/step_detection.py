from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from statistics import fmean, pstdev


@dataclass
class StepEvent:
    timestamp_ms: int
    magnitude: float
    threshold: float
    peak: float


class StepDetector:
    """Adaptive peak detector for IMU acceleration magnitude.

    Pipeline:
    1. Remove gravity by high-pass filtering magnitude (EMA low-pass subtraction).
    2. Track dynamic threshold from rolling standard deviation.
    3. Trigger a step when the high-pass value crosses threshold and local peak is found.
    """

    def __init__(
        self,
        sample_rate_hz: float = 50.0,
        lp_alpha: float = 0.08,
        window_size: int = 40,
        threshold_scale: float = 1.25,
        min_threshold: float = 0.22,
        refractory_ms: int = 260,
    ) -> None:
        self.sample_rate_hz = sample_rate_hz
        self.lp_alpha = lp_alpha
        self.window = deque(maxlen=window_size)
        self.threshold_scale = threshold_scale
        self.min_threshold = min_threshold
        self.refractory_ms = refractory_ms

        self._lp = 0.0
        self._prev_hp = 0.0
        self._last_step_ms = -10_000
        self._peak = 0.0
        self.step_count = 0

    def reset(self) -> None:
        self.window.clear()
        self._lp = 0.0
        self._prev_hp = 0.0
        self._last_step_ms = -10_000
        self._peak = 0.0
        self.step_count = 0

    def update(self, timestamp_ms: int, ax: float, ay: float, az: float) -> StepEvent | None:
        mag = (ax * ax + ay * ay + az * az) ** 0.5

        # Low-pass + high-pass split
        self._lp = self._lp + self.lp_alpha * (mag - self._lp)
        hp = mag - self._lp
        self.window.append(hp)

        if len(self.window) < max(8, self.window.maxlen // 3):
            self._prev_hp = hp
            return None

        sigma = pstdev(self.window)
        threshold = max(self.min_threshold, sigma * self.threshold_scale)

        self._peak = max(self._peak * 0.96, hp)

        crossed_up = self._prev_hp < threshold <= hp
        refractory_ok = (timestamp_ms - self._last_step_ms) >= self.refractory_ms
        peak_ok = self._peak > threshold * 1.08

        evt = None
        if crossed_up and refractory_ok and peak_ok:
            self.step_count += 1
            self._last_step_ms = timestamp_ms
            evt = StepEvent(
                timestamp_ms=timestamp_ms,
                magnitude=mag,
                threshold=threshold,
                peak=self._peak,
            )
            self._peak = 0.0

        self._prev_hp = hp
        return evt

    def debug_state(self) -> dict[str, float]:
        sigma = pstdev(self.window) if len(self.window) > 2 else 0.0
        threshold = max(self.min_threshold, sigma * self.threshold_scale)
        return {
            "low_pass": self._lp,
            "mean_hp": fmean(self.window) if self.window else 0.0,
            "sigma_hp": sigma,
            "threshold": threshold,
            "peak": self._peak,
            "steps": float(self.step_count),
        }
