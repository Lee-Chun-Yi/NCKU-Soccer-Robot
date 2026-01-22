from pathlib import Path

ROOT_DIR = Path(__file__).resolve().parent.parent
CONFIG_DIR = ROOT_DIR / "config"
CALIB_DIR = CONFIG_DIR / "calibration"
DATA_DIR = ROOT_DIR / "recording"

FIELD = [
    [0, 0], [360, 0], [360, 70], [375, 70], [375, 290], [360, 190],
    [360, 260], [0, 260], [0, 190], [-15, 190], [-15, 70], [0, 70],
]

BOUNDARY = [tuple(pt) for pt in FIELD]
_center_x = (FIELD[0][0] + FIELD[1][0]) / 2
_center_y = (FIELD[0][1] + FIELD[7][1]) / 2
CENTER = [int(_center_x), int(_center_y)]
