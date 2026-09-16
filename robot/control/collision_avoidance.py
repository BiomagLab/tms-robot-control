import math
import time
from dataclasses import dataclass
from enum import Enum

import numpy as np


class CollisionStage(Enum):
    UNAVAILABLE = "unavailable"
    CLEAR = "clear"
    APPROACH = "approach"
    WORKING = "working"
    STOP = "stop"


@dataclass(frozen=True)
class RepulsionConfig:
    strength: float
    safety_margin: float
    working_distance: float
    stop_distance: float
    max_offset: float = 2.0
    max_delta_time: float = 0.05
    smoothing: float = 0.2
    measurement_timeout: float = 0.25
    stop_release_distance: float = 2.0
    reaction_time: float = 0.2
    safe_deceleration: float = 500.0
    velocity_smoothing: float = 0.8
    max_closing_speed: float = 300.0

    def __post_init__(self):
        values = (
            self.strength,
            self.safety_margin,
            self.working_distance,
            self.stop_distance,
            self.max_offset,
            self.max_delta_time,
            self.smoothing,
            self.measurement_timeout,
            self.stop_release_distance,
            self.reaction_time,
            self.safe_deceleration,
            self.velocity_smoothing,
            self.max_closing_speed,
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError("Repulsion configuration values must be finite")
        if self.strength < 0:
            raise ValueError("Repulsion strength must not be negative")
        if self.max_offset <= 0 or self.max_delta_time <= 0:
            raise ValueError("Repulsion limits must be positive")
        if not 0 <= self.smoothing < 1:
            raise ValueError("Repulsion smoothing must be in the range [0, 1)")
        if self.measurement_timeout <= 0 or self.stop_release_distance < 0:
            raise ValueError("Measurement timeout must be positive")
        if self.reaction_time < 0 or self.safe_deceleration <= 0:
            raise ValueError("Dynamic braking values must be non-negative")
        if not 0 <= self.velocity_smoothing < 1:
            raise ValueError("Velocity smoothing must be in the range [0, 1)")
        if self.max_closing_speed <= 0:
            raise ValueError("Maximum closing speed must be positive")
        if not 0 <= self.stop_distance < self.working_distance < self.safety_margin:
            raise ValueError(
                "Expected stop_distance < working_distance < safety_margin"
            )

@dataclass(frozen=True)
class RepulsionResult:
    stage: CollisionStage
    magnitude: float
    effective_distance: float
    dynamic_margin: float


@dataclass(frozen=True)
class RepulsionCommand:
    stage: CollisionStage
    magnitude: float
    offset: np.ndarray
    stop_requested: bool


@dataclass(frozen=True)
class CoilDistanceMeasurement:
    distance: float
    direction: np.ndarray
    closing_speed: float
    timestamp: float


class ClosingSpeedEstimator:
    """Estimate radial closing speed from the centers of both coil boxes."""

    def __init__(self, smoothing=0.8, max_closing_speed=300.0, clock=time.monotonic):
        if not 0 <= smoothing < 1:
            raise ValueError("Velocity smoothing must be in the range [0, 1)")
        if not math.isfinite(max_closing_speed) or max_closing_speed <= 0:
            raise ValueError("Maximum closing speed must be positive and finite")
        self.smoothing = smoothing
        self.max_closing_speed = max_closing_speed
        self._clock = clock
        self.reset()

    def update(self, center_a, center_b, timestamp=None) -> float:
        center_a = self._validate_center(center_a)
        center_b = self._validate_center(center_b)
        timestamp = self._clock() if timestamp is None else float(timestamp)
        if not math.isfinite(timestamp):
            raise ValueError("Velocity timestamp must be finite")

        separation = float(np.linalg.norm(center_a - center_b))
        if self._previous_distance is None:
            self._previous_distance = separation
            self._previous_timestamp = timestamp
            return 0.0

        delta_time = timestamp - self._previous_timestamp
        if delta_time < 0:
            raise ValueError("Monotonic clock moved backwards")
        if delta_time > 1e-9:
            radial_speed = (self._previous_distance - separation) / delta_time
            radial_speed = float(
                np.clip(radial_speed, -self.max_closing_speed, self.max_closing_speed)
            )
            if radial_speed <= 0:
                # Do not retain separating motion: a later reversal toward the
                # other coil must be detected on its first valid sample.
                self._filtered_speed = 0.0
            else:
                self._filtered_speed = (
                    self.smoothing * self._filtered_speed
                    + (1 - self.smoothing) * radial_speed
                )

        self._previous_distance = separation
        self._previous_timestamp = timestamp
        return self._filtered_speed

    def reset(self):
        self._previous_distance = None
        self._previous_timestamp = None
        self._filtered_speed = 0.0

    @staticmethod
    def _validate_center(center):
        center = np.asarray(center, dtype=float)
        if center.shape != (3,) or not np.all(np.isfinite(center)):
            raise ValueError("Coil center must contain three finite values")
        return center


class RepulsionField:
    """Compute the repulsion magnitude from the distance between two coils."""

    def __init__(self, config: RepulsionConfig):
        self.config = config

    def compute(self, distance: float, closing_speed: float = 0.0) -> RepulsionResult:
        if not math.isfinite(distance) or distance < 0:
            raise ValueError("Coil distance must be a finite, non-negative value")
        if not math.isfinite(closing_speed) or closing_speed < 0:
            raise ValueError("Closing speed must be a finite, non-negative value")

        dynamic_margin = (
            closing_speed * self.config.reaction_time
            + closing_speed**2 / (2 * self.config.safe_deceleration)
        )
        effective_distance = max(0.0, distance - dynamic_margin)

        if effective_distance <= self.config.stop_distance:
            return RepulsionResult(
                CollisionStage.STOP, 0.0, effective_distance, dynamic_margin
            )

        if effective_distance >= self.config.safety_margin:
            return RepulsionResult(
                CollisionStage.CLEAR, 0.0, effective_distance, dynamic_margin
            )

        if effective_distance <= self.config.working_distance:
            normalized = effective_distance / self.config.working_distance
            magnitude = self.config.strength * math.exp(2 * (1 - normalized))
            return RepulsionResult(
                CollisionStage.WORKING, magnitude, effective_distance, dynamic_margin
            )

        normalized = (self.config.safety_margin - effective_distance) / (
            self.config.safety_margin - self.config.working_distance
        )
        magnitude = self.config.strength * normalized**2
        return RepulsionResult(
            CollisionStage.APPROACH, magnitude, effective_distance, dynamic_margin
        )

    def compute_offset(
        self, distance: float, direction, delta_time: float, closing_speed: float = 0.0
    ) -> RepulsionCommand:
        if not math.isfinite(delta_time) or delta_time < 0:
            raise ValueError("Delta time must be a finite, non-negative value")

        result = self.compute(distance, closing_speed)
        zero_offset = np.zeros(3, dtype=float)

        if result.stage is CollisionStage.STOP:
            return RepulsionCommand(result.stage, result.magnitude, zero_offset, True)
        if result.stage is CollisionStage.CLEAR:
            return RepulsionCommand(result.stage, result.magnitude, zero_offset, False)

        direction = self.normalize_direction(direction)

        effective_delta_time = min(delta_time, self.config.max_delta_time)
        offset = result.magnitude * direction * effective_delta_time

        offset_norm = np.linalg.norm(offset)
        if offset_norm > self.config.max_offset:
            offset *= self.config.max_offset / offset_norm

        return RepulsionCommand(result.stage, result.magnitude, offset, False)

    @staticmethod
    def normalize_direction(direction) -> np.ndarray:
        direction = np.asarray(direction, dtype=float)
        if direction.shape != (3,) or not np.all(np.isfinite(direction)):
            raise ValueError("Repulsion direction must contain three finite values")

        direction_norm = np.linalg.norm(direction)
        if direction_norm <= 1e-9:
            raise ValueError("Repulsion direction must not be a zero vector")

        return direction / direction_norm


class CollisionAvoidanceController:
    """Track coil measurements and produce fail-safe repulsion commands."""

    def __init__(self, config: RepulsionConfig, clock=time.monotonic):
        self.config = config
        self.field = RepulsionField(config)
        self._clock = clock
        self._measurement = None
        self._smoothed_offset = np.zeros(3, dtype=float)
        self._stop_latched = False
        self._monitoring_enabled = False

    @property
    def has_measurement(self) -> bool:
        return self._measurement is not None

    @property
    def stop_latched(self) -> bool:
        return self._stop_latched

    def measurement_is_stale(self) -> bool:
        if self._measurement is None:
            return self._monitoring_enabled
        return self._measurement_age() > self.config.measurement_timeout

    def stop_requested(self) -> bool:
        return self._monitoring_enabled and (
            self._stop_latched or self.measurement_is_stale()
        )

    def activate_monitoring(self):
        """Discard measurements that belong to a previous registration."""
        self._monitoring_enabled = True
        self._measurement = None
        self._stop_latched = False
        self.reset_output()

    def latch_stop(self):
        self._stop_latched = True
        self.reset_output()

    def classify_measurement(self, distance, closing_speed=0.0) -> CollisionStage:
        return self.field.compute(distance, closing_speed).stage

    def update_measurement(
        self,
        distance: float,
        direction,
        closing_speed: float = 0.0,
        timestamp=None,
    ) -> CollisionStage:
        result = self.field.compute(distance, closing_speed)
        timestamp = self._clock() if timestamp is None else float(timestamp)
        if not math.isfinite(timestamp):
            raise ValueError("Measurement timestamp must be finite")
        if timestamp > self._clock():
            raise ValueError("Measurement timestamp must not be in the future")

        if result.stage is CollisionStage.STOP:
            normalized_direction = np.zeros(3, dtype=float)
            self._stop_latched = True
        else:
            normalized_direction = self.field.normalize_direction(direction)
            release_distance = (
                self.config.stop_distance + self.config.stop_release_distance
            )
            if self._stop_latched and distance > release_distance:
                self._stop_latched = False
                self.reset_output()

        self._measurement = CoilDistanceMeasurement(
            distance=distance,
            direction=normalized_direction,
            closing_speed=closing_speed,
            timestamp=timestamp,
        )
        return result.stage

    def compute_command(self, delta_time: float) -> RepulsionCommand:
        if self._measurement is None:
            return self._empty_command(
                CollisionStage.UNAVAILABLE, self._monitoring_enabled
            )

        if self.measurement_is_stale():
            self.reset_output()
            return self._empty_command(CollisionStage.UNAVAILABLE, True)
        if self._stop_latched:
            self.reset_output()
            return self._empty_command(CollisionStage.STOP, True)

        command = self.field.compute_offset(
            self._measurement.distance,
            self._measurement.direction,
            delta_time,
            self._measurement.closing_speed,
        )
        if command.stage is CollisionStage.CLEAR:
            self.reset_output()
            return command

        self._smoothed_offset = (
            self.config.smoothing * self._smoothed_offset
            + (1 - self.config.smoothing) * command.offset
        )
        return RepulsionCommand(
            command.stage,
            command.magnitude,
            self._smoothed_offset.copy(),
            command.stop_requested,
        )

    def reset_output(self):
        self._smoothed_offset.fill(0)

    def _measurement_age(self) -> float:
        age = self._clock() - self._measurement.timestamp
        if age < 0:
            raise ValueError("Monotonic clock moved backwards")
        return age

    @staticmethod
    def _empty_command(stage, stop_requested):
        return RepulsionCommand(
            stage=stage,
            magnitude=0.0,
            offset=np.zeros(3, dtype=float),
            stop_requested=stop_requested,
        )
