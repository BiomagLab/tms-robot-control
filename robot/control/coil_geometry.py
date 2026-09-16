import time
from dataclasses import dataclass

import numpy as np
from scipy.optimize import lsq_linear
from scipy.spatial.transform import Rotation

from robot.control.collision_avoidance import ClosingSpeedEstimator, CollisionStage

DEFAULT_COIL_HALF_THICKNESS_MM = 7.0


@dataclass(frozen=True)
class OrientedBoundingBox:
    """An oriented box represented by its center and three half-axis vectors."""

    center: np.ndarray
    half_axes: np.ndarray

    def __post_init__(self):
        center = np.asarray(self.center, dtype=float)
        half_axes = np.asarray(self.half_axes, dtype=float)

        if center.shape != (3,):
            raise ValueError("OBB center must contain three values")
        if half_axes.shape != (3, 3):
            raise ValueError("OBB half_axes must have shape (3, 3)")
        if not np.all(np.isfinite(center)) or not np.all(np.isfinite(half_axes)):
            raise ValueError("OBB values must be finite")
        if abs(np.linalg.det(half_axes)) <= 1e-9:
            raise ValueError("OBB half-axes must define a non-degenerate volume")

        object.__setattr__(self, "center", center.copy())
        object.__setattr__(self, "half_axes", half_axes.copy())

    def transformed(self, position, rotation) -> "OrientedBoundingBox":
        position = np.asarray(position, dtype=float)
        rotation = np.asarray(rotation, dtype=float)
        if position.shape != (3,) or rotation.shape != (3, 3):
            raise ValueError("Expected a 3D position and a 3x3 rotation")
        if not np.all(np.isfinite(position)) or not np.all(np.isfinite(rotation)):
            raise ValueError("OBB transformation values must be finite")

        return OrientedBoundingBox(
            center=rotation @ self.center + position,
            half_axes=(rotation @ self.half_axes.T).T,
        )


@dataclass(frozen=True)
class CoilCollisionMeasurement:
    distance: float
    object_id_a: int
    object_id_b: int
    closest_point_a: np.ndarray
    closest_point_b: np.ndarray
    brake_direction_a: np.ndarray
    brake_direction_b: np.ndarray
    box_a: OrientedBoundingBox
    box_b: OrientedBoundingBox

    def direction_for(self, object_id: int) -> np.ndarray:
        if object_id == self.object_id_a:
            return self.brake_direction_a.copy()
        if object_id == self.object_id_b:
            return self.brake_direction_b.copy()
        raise ValueError(f"Tracker object ID {object_id} is not a collision coil")

    def box_for(self, object_id: int) -> OrientedBoundingBox:
        if object_id == self.object_id_a:
            return self.box_a
        if object_id == self.object_id_b:
            return self.box_b
        raise ValueError(f"Tracker object ID {object_id} is not a collision coil")


class CoilCollisionCalculator:
    """Calculate collision data for two registered tracker coils."""

    def __init__(
        self,
        registrations,
        half_thickness=DEFAULT_COIL_HALF_THICKNESS_MM,
        lateral_expansion=0.0,
        face_expansion=0.0,
    ):
        if not isinstance(registrations, dict) or len(registrations) != 2:
            raise ValueError("Collision calculation requires exactly two coils")

        self._coils = []
        object_ids = set()
        for coil_name, registration in registrations.items():
            if not isinstance(registration, dict):
                raise ValueError(f"Coil {coil_name} has an invalid registration")
            object_id = registration.get("obj_id")
            if (
                not isinstance(object_id, int)
                or isinstance(object_id, bool)
                or object_id < 0
            ):
                raise ValueError(f"Coil {coil_name} has an invalid tracker object ID")
            if object_id in object_ids:
                raise ValueError("Collision coils must use distinct tracker object IDs")
            object_ids.add(object_id)
            self._coils.append(
                (
                    object_id,
                    coil_box_from_registration(
                        registration,
                        half_thickness,
                        lateral_expansion,
                        face_expansion,
                    ),
                )
            )

    @property
    def object_ids(self):
        return tuple(coil[0] for coil in self._coils)

    def measure(self, tracker_coordinates) -> CoilCollisionMeasurement:
        world_boxes = []
        for object_id, local_box in self._coils:
            try:
                pose = np.asarray(tracker_coordinates[object_id], dtype=float)
            except (IndexError, TypeError, ValueError) as error:
                raise ValueError(
                    f"Tracker pose for coil object {object_id} is unavailable"
                ) from error

            if pose.ndim != 1 or pose.size < 6 or not np.all(np.isfinite(pose[:6])):
                raise ValueError(f"Tracker pose for coil object {object_id} is invalid")

            rotation = Rotation.from_euler("ZYX", pose[3:6], degrees=True).as_matrix()
            world_boxes.append(local_box.transformed(pose[:3], rotation))

        return measure_obb_distance(
            world_boxes[0], world_boxes[1], self.object_ids[0], self.object_ids[1]
        )


class CoilCollisionTracker:
    """Own coil registrations and motion history, without commanding a robot."""

    def __init__(self, controller, clock=time.monotonic):
        self.controller = controller
        self.calculator = None
        self.coil_index = None
        self._latest_coil_box = None
        self._latest_head_center = None
        self.speed_estimator = ClosingSpeedEstimator(
            smoothing=controller.config.velocity_smoothing,
            max_closing_speed=controller.config.max_closing_speed,
            clock=clock,
        )

    def set_registrations(self, registrations, coil_index, **geometry_config):
        """Validate all registration data before replacing the current geometry."""
        if (
            not isinstance(coil_index, int)
            or isinstance(coil_index, bool)
            or coil_index < 0
        ):
            raise ValueError("Own coil tracker object ID must be a non-negative integer")
        calculator = CoilCollisionCalculator(registrations, **geometry_config)
        if coil_index not in calculator.object_ids:
            raise ValueError("Own coil is not present in the collision registrations")
        self.calculator = calculator
        self.coil_index = coil_index
        self.speed_estimator.reset()
        self._latest_coil_box = None
        self._latest_head_center = None
        self.controller.activate_monitoring()

    def set_coil_index(self, coil_index):
        if (
            not isinstance(coil_index, int)
            or isinstance(coil_index, bool)
            or coil_index < 0
        ):
            raise ValueError("Own coil tracker object ID must be a non-negative integer")
        if self.calculator is None:
            return
        if coil_index not in self.calculator.object_ids:
            raise ValueError("Own coil is not present in the collision registrations")
        self.coil_index = coil_index
        self.speed_estimator.reset()
        self._latest_coil_box = None
        self._latest_head_center = None
        self.controller.activate_monitoring()

    def update_from_tracker_poses(
        self, poses, visibilities, tracker_to_robot, get_head_center, timestamp=None
    ):
        """Return (stage, distance), or None when no new measurement is available.

        get_head_center is evaluated only when repulsion needs a head constraint.
        It must return the current frame's head center in robot coordinates.
        Invalid frames raise an error for the caller to stop the robot.
        """
        if self.calculator is None:
            return None
        try:
            if any(not bool(visibilities[i]) for i in self.calculator.object_ids):
                # Keep the previous timestamp so the controller's watchdog expires.
                self.speed_estimator.reset()
                self._latest_coil_box = None
                self._latest_head_center = None
                return None
            measurement = self.calculator.measure(poses)
            closing_speed = self.speed_estimator.update(
                measurement.box_a.center, measurement.box_b.center, timestamp
            )
            stage = self.controller.classify_measurement(
                measurement.distance, closing_speed
            )
            direction = measurement.direction_for(self.coil_index)
            if stage not in (CollisionStage.STOP, CollisionStage.CLEAR):
                direction = direction_from_tracker_to_robot(direction, tracker_to_robot)
                head_center = get_head_center()
                coil_box = box_from_tracker_to_robot(
                    measurement.box_for(self.coil_index), tracker_to_robot
                )
                direction = constrain_direction_away_from_head(
                    direction, coil_box, head_center
                )
                self._latest_coil_box = coil_box
                self._latest_head_center = np.asarray(head_center, dtype=float).copy()
            else:
                self._latest_coil_box = None
                self._latest_head_center = None
            stage = self.controller.update_measurement(
                measurement.distance, direction, closing_speed, timestamp
            )
            return stage, measurement.distance
        except (IndexError, TypeError, ValueError, RuntimeError):
            self.speed_estimator.reset()
            self._latest_coil_box = None
            self._latest_head_center = None
            raise

    def constrain_repulsion_offset(self, offset):
        offset = np.asarray(offset, dtype=float)
        magnitude = float(np.linalg.norm(offset))
        if magnitude <= 1e-9:
            return offset
        if self._latest_coil_box is None or self._latest_head_center is None:
            raise ValueError("Current collision geometry is unavailable")
        direction = constrain_direction_away_from_head(
            offset / magnitude, self._latest_coil_box, self._latest_head_center
        )
        return direction * magnitude


def coil_box_from_registration(
    registration,
    half_thickness=DEFAULT_COIL_HALF_THICKNESS_MM,
    lateral_expansion=0.0,
    face_expansion=0.0,
) -> OrientedBoundingBox:
    """Build a marker-local coil OBB from an InVesalius coil registration."""

    if not np.isfinite(half_thickness) or half_thickness <= 0:
        raise ValueError("Coil half-thickness must be a positive finite value")
    expansions = (lateral_expansion, face_expansion)
    if not all(np.isfinite(value) and value >= 0 for value in expansions):
        raise ValueError("Coil box expansions must be finite and non-negative")
    if not isinstance(registration, dict):
        raise ValueError("Coil registration must be a dictionary")

    try:
        fiducials = np.asarray(registration["fiducials"], dtype=float)
        orientations = np.asarray(registration["orientations"], dtype=float)
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError(
            "Coil registration has invalid fiducials or orientations"
        ) from error

    if fiducials.ndim != 2 or fiducials.shape[0] < 4 or fiducials.shape[1] < 3:
        raise ValueError("Coil registration must contain four 3D fiducials")
    if orientations.ndim != 2 or orientations.shape[0] < 4 or orientations.shape[1] < 3:
        raise ValueError("Coil registration must contain four orientations")
    if not np.all(np.isfinite(fiducials)) or not np.all(np.isfinite(orientations)):
        raise ValueError("Coil registration values must be finite")

    left, right, anterior, marker_position = fiducials[:4, :3]
    marker_orientation = orientations[3, :3]
    center = (left + right) / 2
    half_width = (right - left) / 2
    half_depth = anterior - center

    normal = np.cross(half_width, half_depth)
    normal_norm = np.linalg.norm(normal)
    if normal_norm <= 1e-9:
        raise ValueError("Coil registration fiducials must not be collinear")
    half_width = _expand_half_axis(half_width, lateral_expansion)
    half_depth = _expand_half_axis(half_depth, lateral_expansion)
    half_normal = normal / normal_norm * (half_thickness + face_expansion)

    marker_to_tracker = Rotation.from_euler(
        "ZYX", marker_orientation, degrees=True
    ).as_matrix()
    tracker_to_marker = marker_to_tracker.T

    return OrientedBoundingBox(
        center=tracker_to_marker @ (center - marker_position),
        half_axes=np.array(
            [
                tracker_to_marker @ half_width,
                tracker_to_marker @ half_depth,
                tracker_to_marker @ half_normal,
            ]
        ),
    )


def _expand_half_axis(axis, expansion):
    norm = np.linalg.norm(axis)
    if norm <= 1e-9:
        raise ValueError("Coil registration half-axes must have positive length")
    return axis * ((norm + expansion) / norm)


def measure_obb_distance(
    box_a: OrientedBoundingBox,
    box_b: OrientedBoundingBox,
    object_id_a: int = 0,
    object_id_b: int = 1,
) -> CoilCollisionMeasurement:
    """Return the minimum surface distance and separation directions for two OBBs."""

    matrix = np.column_stack((box_a.half_axes.T, -box_b.half_axes.T))
    target = box_b.center - box_a.center
    solution = lsq_linear(matrix, target, bounds=(-1.0, 1.0), tol=1e-12)
    if not solution.success:
        raise RuntimeError(f"Unable to calculate OBB distance: {solution.message}")

    point_a = box_a.center + box_a.half_axes.T @ solution.x[:3]
    point_b = box_b.center + box_b.half_axes.T @ solution.x[3:]
    separation = point_a - point_b
    distance = float(np.linalg.norm(separation))

    if distance > 1e-9:
        direction_a = separation / distance
        direction_b = -direction_a
    else:
        direction_a = np.zeros(3, dtype=float)
        direction_b = np.zeros(3, dtype=float)
        distance = 0.0

    return CoilCollisionMeasurement(
        distance=distance,
        object_id_a=object_id_a,
        object_id_b=object_id_b,
        closest_point_a=point_a,
        closest_point_b=point_b,
        brake_direction_a=direction_a,
        brake_direction_b=direction_b,
        box_a=box_a,
        box_b=box_b,
    )


def direction_from_tracker_to_robot(direction, tracker_to_robot) -> np.ndarray:
    """Transform a direction from tracker space to the robot BASE system."""

    direction = np.asarray(direction, dtype=float)
    if direction.shape != (3,) or not np.all(np.isfinite(direction)):
        raise ValueError("Brake direction must contain three finite values")

    matrix = _robot_affine(tracker_to_robot)

    transformed = matrix[:3, :3] @ direction
    norm = np.linalg.norm(transformed)
    if norm <= 1e-9:
        return np.zeros(3, dtype=float)
    return transformed / norm


def box_from_tracker_to_robot(
    box: OrientedBoundingBox, tracker_to_robot
) -> OrientedBoundingBox:
    """Transform a tracker-space box to the robot BASE coordinate system."""

    matrix = _robot_affine(tracker_to_robot)
    return box.transformed(matrix[:3, 3], matrix[:3, :3])


def closest_point_on_obb(box: OrientedBoundingBox, point) -> np.ndarray:
    """Return the closest point on or inside a box to an arbitrary point."""

    point = np.asarray(point, dtype=float)
    if point.shape != (3,) or not np.all(np.isfinite(point)):
        raise ValueError("Reference point must contain three finite values")

    solution = lsq_linear(
        box.half_axes.T,
        point - box.center,
        bounds=(-1.0, 1.0),
        tol=1e-12,
    )
    if not solution.success:
        raise RuntimeError(f"Unable to calculate closest box point: {solution.message}")
    return box.center + box.half_axes.T @ solution.x


def constrain_direction_away_from_head(
    coil_direction, coil_box: OrientedBoundingBox, head_center
) -> np.ndarray:
    """Remove any component of coil repulsion that points toward the head."""

    coil_direction = _normalize_direction(coil_direction, "Coil repulsion direction")
    head_center = np.asarray(head_center, dtype=float)
    closest_coil_point = closest_point_on_obb(coil_box, head_center)
    head_direction = _normalize_direction(
        closest_coil_point - head_center, "Head avoidance direction"
    )

    head_component = float(np.dot(coil_direction, head_direction))
    if head_component >= 0:
        return coil_direction

    safe_direction = coil_direction - head_component * head_direction
    safe_norm = np.linalg.norm(safe_direction)
    if safe_norm <= 1e-6:
        raise ValueError("No repulsion direction can separate the coils away from the head")
    return safe_direction / safe_norm


def _normalize_direction(direction, name):
    direction = np.asarray(direction, dtype=float)
    if direction.shape != (3,) or not np.all(np.isfinite(direction)):
        raise ValueError(f"{name} must contain three finite values")
    norm = np.linalg.norm(direction)
    if norm <= 1e-9:
        raise ValueError(f"{name} must not be zero")
    return direction / norm


def _robot_affine(tracker_to_robot):
    if isinstance(tracker_to_robot, (tuple, list)) and len(tracker_to_robot) == 3:
        matrix = np.asarray(tracker_to_robot[2], dtype=float)
    else:
        matrix = np.asarray(tracker_to_robot, dtype=float)
        try:
            matrix = matrix.reshape(-1, 4)
        except ValueError as error:
            raise ValueError("Tracker-to-robot matrix has an invalid shape") from error
        if matrix.shape == (12, 4):
            matrix = matrix[8:12]

    if matrix.shape != (4, 4) or not np.all(np.isfinite(matrix)):
        raise ValueError("Tracker-to-robot affine matrix must have shape (4, 4)")
    return matrix
