DISPLAY_POA = True

TEMP_FILE = "tmp"

MAX_NAVIGATION_UPDATE_GAP_SECONDS = 1

# Site config
#
# rx_offset, ry_offset, rz_offset:
#   The offset to align the coil and robot end effector (TODO: unit?)

SITE_CONFIG = {
    "usp_coil": {
        "rx_offset": 0,
        "ry_offset": 0,
        "rz_offset": 0,
    },
    "usp_neurosoft": {
        "rx_offset": -90,
        "ry_offset": 0,
        "rz_offset": -30,
    },
    "aalto": {
        "rx_offset": 0,
        "ry_offset": 0,
        "rz_offset": 0,
    },
    "tubingen": {
        "rx_offset": 0,
        "ry_offset": 0,
        "rz_offset": 0,
    },
    "default": {
        "rx_offset": 0,
        "ry_offset": 0,
        "rz_offset": 0,
    },
}

# Robot config
#
# General:
#   Working space: TODO (in mm)
#   Sleep: The time between two cycles of the main loop (in seconds).
#   Waiting time: The time to wait after a movement (in seconds).
#
# Head motion:
#   Head velocity threshold: TODO (in mm/s)
#
# Tuning motion:
#   Target tuning threshold distance: TODO (unit?)
#   Target tuning threshold angle: TODO (in degrees)
#
# Arc motion:
#   Arc threshold distance: TODO (unit?)
#   Arc threshold angle: TODO (in degrees)
#   Arc bezier curve step: Step size between 0 and 1; 0.1 generates 10 points, 0.01 generates 100 points along the curve path.
#   Threshold to update target arc: TODO (in mm)
#   Versor scale factor: TODO
#   Middle arc scale factor: TODO
#
# Robot tracker registration:
#   Transformation matrix threshold: TODO (in mm)
#
# Force and torque sensor:
#   Force sensor threshold: TODO (in Newtons)
#   Sensor scale threshold: TODO (as percentage of initial force)
FORCE_SENSOR_SAFETY_THRESHOLD_N = 30.0

ROBOT_CONFIG = {
    "ur": {
        # General
        # Robot working space is defined as 800 mm in Elfin 5 manual. For safety, the value is
        # reduced by 5%. For debugging, feel free to use 1000 mm.
        "working_space_radius": 1000,
        "sleep": 0.005,
        # Head motion
        "head_velocity_threshold": 15,
        # Tuning motion
        "distance_threshold_for_tuning": 30,
        "angular_distance_threshold_for_tuning": 5,
        # Arc motion
        "distance_threshold_for_arc_motion": 80,
        "angular_distance_threshold_for_arc_motion": 30,
        "arc_bezier_curve_step": None,
        "threshold_to_update_arc_motion_target": 40,
        "versor_scale_factor": 50,
        "middle_arc_scale_factor": 1.5,
        # Robot tracker registration
        "transformation_matrix_threshold": 1,
        # Force and torque sensor
        "force_sensor_threshold": 10,
        "force_sensor_scale_threshold": 30,
    },
    "dobot": {
        # General
        "working_space_radius": 1500,
        "sleep": 0.005,
        # Head motion
        "head_velocity_threshold": 60,
        # Tuning motion
        "distance_threshold_for_tuning": 50,
        "angular_distance_threshold_for_tuning": 15,
        # Arc motion
        "distance_threshold_for_arc_motion": 100,
        "angular_distance_threshold_for_arc_motion": 45,
        "arc_bezier_curve_step": 0.4,
        "threshold_to_update_arc_motion_target": 40,
        "versor_scale_factor": 50,
        "middle_arc_scale_factor": 4,
        # Robot tracker registration
        "transformation_matrix_threshold": 1,
        # Force and torque sensor
        "force_sensor_threshold": 10,
        "force_sensor_scale_threshold": 100,
    },
    "elfin": {
        # General
        # Robot working space is defined as 800 mm in Elfin 5 manual. For safety, the value is
        # reduced by 5%. For debugging, feel free to use 1000 mm.
        "working_space_radius": 850,
        "sleep": 0.01,
        # Head motion
        "head_velocity_threshold": 120,
        # Tuning motion
        "distance_threshold_for_tuning": 30,
        "angular_distance_threshold_for_tuning": 5,
        # Arc motion
        "distance_threshold_for_arc_motion": 80,
        "angular_distance_threshold_for_arc_motion": 30,
        "arc_bezier_curve_step": None,
        "threshold_to_update_arc_motion_target": 40,
        "versor_scale_factor": 50,
        "middle_arc_scale_factor": 1.5,
        # Robot tracker registration
        "transformation_matrix_threshold": 1,
        # Force and torque sensor
        "force_sensor_threshold": 10,
        "force_sensor_scale_threshold": 30,
    },
}

# Coil-to-coil collision avoidance. Distances and offsets are expressed in mm.
# These initial values preserve the shape of the prototype curve, while using a
# practical stop threshold that must be validated on the physical setup.
COLLISION_AVOIDANCE_CONFIG = {
    "strength": 25.0,
    "safety_margin": 22.0,
    "working_distance": 12.0,
    "stop_distance": 5.0,
    "max_offset": 2.0,
    "max_delta_time": 0.05,
    "smoothing": 0.2,
    "measurement_timeout": 0.25,
    "stop_release_distance": 2.0,
    # Dynamic clearance = speed * reaction_time + speed^2 / (2 * deceleration).
    # Initial physical assumptions; validate them with measured stopping tests.
    "reaction_time": 0.2,
    "safe_deceleration": 500.0,
    "velocity_smoothing": 0.8,
    "max_closing_speed": 300.0,
}

# Per-face expansion of the registered coil bounding box, in mm. Lateral
# expansion applies to both in-plane axes; face expansion applies along the
# normal to the coil plane, on its front and back faces.
COIL_COLLISION_GEOMETRY_CONFIG = {
    "half_thickness": 7.0,
    "lateral_expansion": 1.5,
    "face_expansion": 3.0,
}

# Publisher messages from invesalius to any robot
# It's important these message to be in the PUB_MESSAGES list too
PUB_MESSAGES_WL = [
    "From Neuronavigation: Update tracker poses",
]

# Publisher messages from invesalius
PUB_MESSAGES = [
    "Neuronavigation to Robot: Connect to robot",
    "Neuronavigation to Robot: Set target",
    "Neuronavigation to Robot: Unset target",
    "Neuronavigation to Robot: Set tracker fiducials",
    "From Neuronavigation: Update tracker poses",
    "Neuronavigation to Robot: Collect coordinates for the robot transformation matrix",
    "Neuronavigation to Robot: Reset coordinates collection for the robot transformation matrix",
    "Neuronavigation to Robot: Estimate robot transformation matrix",
    "Neuronavigation to Robot: Set robot transformation matrix",
    "From Neuronavigation: Coil at target",
    "Neuronavigation to Robot: Update displacement to target",
    "Neuronavigation to Robot: Set objective",
    "Neuronavigation to Robot: Set free drive",
    "Neuronavigation to Robot: Check connection robot",
    "Neuronavigation to Robot: Pressure set point",
    "Neuronavigation to Robot: Update config",
    "Neuronavigation to Robot: Update robot control pid factors",
    "Neuronavigation to Robot: Request config",
    "Dashboard to Robot: Request pid factors",
    "Neuronavigation to Robot: Set coil index",
    "Neuronavigation to Robot: Reset errors",
    "Neuronavigation to Robot: Set coil collision registrations",
]

FUNCTION_CONNECT_TO_ROBOT = 0
FUNCTION_SET_TARGET = 1
FUNCTION_UNSET_TARGET = 2
FUNCTION_SET_TRACKER_FIDUCIALS = 3
FUNCTION_UPDATE_TRACKER_POSES = 4
FUNCTION_COLLECT_COORDINATES_TO_ROBOT_MATRIX = 5
FUNCTION_RESET_ROBOT_MATRIX = 6
FUNCTION_ROBOT_MATRIX_ESTIMATION = 7
FUNCTION_SET_ROBOT_TRANSFORMATION_MATRIX = 8
FUNCTION_COIL_AT_TARGET = 9
FUNCTION_UPDATE_DISPLACEMENT_TO_TARGET = 10
FUNCTION_SET_OBJECTIVE = 11
FUNCTION_SET_FREE_DRIVE = 12
FUNCTION_CHECK_CONNECTION = 13
FUNCTION_SET_PRESSURE_SET_POINT = 14
FUNCTION_UPDATE_CONFIG = 15
FUNCTION_UPDATE_PID = 16
FUNCTION_REQUEST_CONFIG = 17
FUNCTION_REQUEST_PID = 18
FUNCTION_SET_COIL_INDEX = 19
FUNCTION_CLEAN_ERRORS = 20
FUNCTION_SET_COLLISION_REGISTRATIONS = 21
