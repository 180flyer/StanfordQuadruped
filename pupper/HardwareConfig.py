"""
Per-robot configuration file that is particular to each individual robot, not just the type of robot.
"""
import numpy as np


MICROS_PER_RAD = 11.333 * 180.0 / np.pi  # Must be calibrated
NEUTRAL_ANGLE_DEGREES = np.array(
    [[-7, -2, 2, -1], [18, 55, 33, 50], [-46, -37, -40, -39]]
)

PS4_COLOR = {"red": 0, "blue": 0, "green": 255}
PS4_DEACTIVATED_COLOR = {"red": 0, "blue": 0, "green": 50}