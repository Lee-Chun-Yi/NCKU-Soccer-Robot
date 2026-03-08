import cv2
import math
import time
import enum
import numpy as np
import sys
import os
import base64
from pathlib import Path

ROOT_DIR = Path(__file__).resolve().parent.parent.parent
SRC_DIR = ROOT_DIR / "src"
for extra in [ROOT_DIR, SRC_DIR, SRC_DIR / "vision", SRC_DIR / "strategies", SRC_DIR / "nrf", SRC_DIR / "utils"]:
    extra_str = str(extra)
    if extra_str not in sys.path:
        sys.path.insert(0, extra_str)

# Local package imports (support both packaged and script entry)
try:
    from simulator.simulator_appearance import *
except ImportError:
    from simulator_appearance import *


def _load_main_module():
    """Find a loaded main module that exposes BOUNDARY/CENTER."""
    for name in ("src.main", "main", "__main__"):
        mod = sys.modules.get(name)
        if mod and hasattr(mod, "BOUNDARY") and hasattr(mod, "CENTER"):
            return mod
    class Dummy:
        # Default field if no main module is loaded (standalone simulator)
        BOUNDARY = [
            (0, 0), (360, 0), (360, 70), (375, 70), (375, 190), (360, 190),
            (360, 260), (0, 260), (0, 190), (-15, 190), (-15, 70), (0, 70),
        ]
        CENTER = [180, 130]
        @staticmethod
        def Initialize(*_args, **_kwargs):
            return
        @staticmethod
        def strategy_update_field(*_args, **_kwargs):
            return
    return Dummy()


st_A = _load_main_module()
BOUNDARY = list(getattr(st_A, "BOUNDARY", [(0, 0)] * 12))
CENTER = list(getattr(st_A, "CENTER", [0, 0]))
ORIG_BOUNDARY = list(BOUNDARY)
ORIG_CENTER = list(CENTER)


class _DummyRobot:
    def __init__(self, pos=None, direction=None):
        self.pos = pos if pos is not None else [float(CENTER[0]), float(CENTER[1])]
        self.dir = direction if direction is not None else [1.0, 0.0]


# Ensure required attributes exist to avoid runtime errors
if not hasattr(st_A, "Player"):
    st_A.Player = _DummyRobot()
if not hasattr(st_A, "OPPO"):
    st_A.OPPO = _DummyRobot([float(CENTER[0]) + 50.0, float(CENTER[1])], [-1.0, 0.0])
if not hasattr(st_A, "ball_pos"):
    st_A.ball_pos = [int(CENTER[0]), int(CENTER[1])]
if not hasattr(st_A, "rob"):
    def _rob(idx):
        if idx == 0:
            return st_A.Player
        elif idx == 1:
            return st_A.OPPO
        else:
            return _DummyRobot()
    st_A.rob = _rob
if not hasattr(st_A, "Update_Robo_Info"):
    def _noop_update_robo_info(*args, **kwargs):
        return
    st_A.Update_Robo_Info = _noop_update_robo_info

robo_num = 2


FRAME_WIDTH, FRAME_HEIGHT = 900, 600
# Offset will be recalculated to center the field inside the frame
OFFSET = [0, 0]
SCALE = 1

# Game parameter
Side = -1 #1:Left A  Right B  #-1:Left B  Right A
CMD_FPS = 2
CAM_FPS = 15

# Robot movement parameter
Robo_rotate_speed = 4.5*math.pi/180 #radius change
'''
    the step size of a robot for each command
    [fast_for, for, back, move_left, move_right, fast_move_l, fast_move_r]
'''
Robo_move = [1.2*2, 0.66, 0.66, 0.53, 0.53, 5.3, 5.3]  # [4, 3, 3, 4, 4, 6, 6]

# Ball simulation parameter
Ball_bump_speed = 3         # Bump speed when robot walking #走路碰到時起使速度
Ball_kicked_speed = 4*2       # Robot kick speed #提及時起使速度
Ball_n_acce = 0.2/1           # Ball moving deacceleration #球的移動減速度
Ball_passing_speed = 2      # Robot pass speed #經過碰到時
Ball_rotbump_speed = 4      # Bump speed when robot turning #旋轉踢到時的速度

# Simulator debug option
Cursor_Position_Show = 0

# Simulator inner parameter
ball_direction = [1.0, 0.0]
ball_pos = [0, 0] 
ball_speed = 0 
FRAME_FPS = 30 
Kickable_distance = 70 
mode = 1 
Mouse_Position = [0, 0]
CMD = ['N1', 'N1','N1']
# 因為使用機器人的class，裡面有定義方向
# Robo_Direction = [[1.0, 0.0], [1.0, 0.0], [1.0, 0.0], [-1.0, 0.0], [-1.0, 0.0], [-1.0, 0.0]]
Robo_kick_angle_threshold = 60*math.pi/180 
Robo_rotate_rect = [None, None, None, None, None, None]
Robo_rotate_vert = [None, None, None, None, None, None]
Setup_Index = 0 
_realtime_initialized = False
_realtime_field_params = {}
_realtime_empty_field = None
_snapshot_mode = False
_snapshot_start = False
_snapshot_initial = {}
# Realtime Tk view state (used by main.py)
_rt_win = None
_rt_running = False
_rt_overlay_enabled = True
_rt_canvas = None
_rt_last_team = []
_rt_last_dirs = []
_rt_last_ball = []
_rt_strategy_mod = None
_rt_provider = None
_rt_root = None
# global field image for collision/bounds
empty_field = None

# The simulation of object is simulated with update frequency @FRAME_FPS
# In practice, the camera has maximum update frequency of 30 fps. You can set this parameter with @CAM_FPS
# Also, the sending frequency in practice will not be 30 fps because it is too frequent for robot to reacat.
# YOu can modify the sending frequency in @CMD_FPS
# If you modify the value, you should modify object speed with appropriate scalar. Implement by yourself.
STRATEGY_UP, SEND_UP = int(FRAME_FPS/CAM_FPS), int(FRAME_FPS/CMD_FPS)

class MOVE(enum.Enum):
    NONE = 'N'
    FORWARD = 'w'
    BACKWARD = 's'
    TRUN_LEFT = 'q'
    TRUN_RIGHT = 'e'
    MOVE_LEFT = 'a'
    MOVE_RIGHT = 'd'
    F_FORWARD = 'W'
    F_MOVE_LEFT = 'A'
    F_MOVE_RIGHT = 'D'
    F_TRUN_LEFT = 'Q'
    F_TRUN_RIGHT = 'E'
    RFSHOOT = 'i'
    LFSHOOT = 'u'
        
    RSSHOOT = 'j'
    LSSHOOT = 'h'
    RBSHOOT = 'n'
    LBSHOOT = 'b'
    RPASS = 'p'
    LPASS = 'o'
    F_DEFENSE = 'Y'
    L_DEFENSE = 'f'
    R_DEFENSE = 'g'
    DEFENSE = 'y' 
    REST = 'r' 
    STAND = 'R'   
    STOP = 'x'  
    START = 'z'
    PK_MODE = 'c'

############################# Simulator Function ##################################

def draw_empty(frame,FB_x, FB_y, PK_x, Penalty_y, Center_Circle, GA_x, GA_y):#frame 為生成圖檔的名子
    """透過已知的資訊生成圖檔(名字叫frame)"""
    # Bounding box for clean edges
    xs = [p[0] for p in BOUNDARY]
    ys = [p[1] for p in BOUNDARY]
    x0, x1 = min(xs), max(xs)
    y0, y1 = min(ys), max(ys)

    #mark center
    cv2.rectangle(frame, (int(CENTER[0])-CenterHalfSize, int(CENTER[1])-CenterHalfSize), (int(CENTER[0])+CenterHalfSize, int(CENTER[1])+CenterHalfSize), Line_Color, -1)
    #mark middle line
    cv2.line(frame, (CENTER[0], y0), (CENTER[0], y1), Line_Color, 3)
    #mark goal line (use bbox instead of irregular boundary points)
    cv2.line(frame, (x0, y0), (x1, y0), (0, 0, 255), 5)
    cv2.line(frame, (x1, y0), (x1, y1), (0, 0, 255), 5)
    cv2.line(frame, (x1, y1), (x0, y1), (0, 0, 255), 5)
    cv2.line(frame, (x0, y1), (x0, y0), (0, 0, 255), 5)

    #Build FK FB PK point
    cv2.circle(frame, (int(CENTER[0]+FB_x), (int(CENTER[1]))), 5, Line_Color, -1, 8, 0)
    cv2.circle(frame, (int(CENTER[0]+FB_x), (int(CENTER[1]+FB_y))), 3, Line_Color, -1, 8, 0)
    cv2.circle(frame, (int(CENTER[0]+FB_x), (int(CENTER[1]-FB_y))), 3, Line_Color, -1, 8, 0)
    cv2.circle(frame, (int(CENTER[0]-FB_x), (int(CENTER[1]))), 5, Line_Color, -1, 8, 0)
    cv2.circle(frame, (int(CENTER[0]-FB_x), (int(CENTER[1]+FB_y))), 3, Line_Color, -1, 8, 0)
    cv2.circle(frame, (int(CENTER[0]-FB_x), (int(CENTER[1]-FB_y))), 3, Line_Color, -1, 8, 0)
    cv2.circle(frame, (int(CENTER[0]+PK_x), (int(CENTER[1]))), 5, Line_Color, -1, 8, 0)
    cv2.circle(frame, (int(CENTER[0]-PK_x), (int(CENTER[1]))), 5, Line_Color, -1, 8, 0)
    #Build Penalty Area
    cv2.line(frame,(int(CENTER[0]+PK_x),(int(CENTER[1]+Penalty_y))),(int(CENTER[0]+PK_x),(int(CENTER[1]-Penalty_y))), Line_Color,3)
    cv2.line(frame,(int(CENTER[0]-PK_x),(int(CENTER[1]+Penalty_y))),(int(CENTER[0]-PK_x),(int(CENTER[1]-Penalty_y))), Line_Color,3)
    cv2.line(frame,(int(CENTER[0]+PK_x),(int(CENTER[1]+Penalty_y))),(int(x1*0.96),(int(CENTER[1]+Penalty_y))), Line_Color,3)
    cv2.line(frame,(int(CENTER[0]+PK_x),(int(CENTER[1]-Penalty_y))),(int(x1*0.96),(int(CENTER[1]-Penalty_y))), Line_Color,3)
    cv2.line(frame,(int(CENTER[0]-PK_x),(int(CENTER[1]+Penalty_y))),(int(x0*1.04),(int(CENTER[1]+Penalty_y))), Line_Color,3)
    cv2.line(frame,(int(CENTER[0]-PK_x),(int(CENTER[1]-Penalty_y))),(int(x0*1.04),(int(CENTER[1]-Penalty_y))), Line_Color,3)
    #Build Goal Area
    cv2.line(frame,(int(CENTER[0]+GA_x),(int(CENTER[1]+GA_y))),(int(CENTER[0]+GA_x),(int(CENTER[1]-GA_y))), Line_Color,3)
    cv2.line(frame,(int(CENTER[0]-GA_x),(int(CENTER[1]+GA_y))),(int(CENTER[0]-GA_x),(int(CENTER[1]-GA_y))), Line_Color,3)
    cv2.line(frame,(int(CENTER[0]+GA_x),(int(CENTER[1]+GA_y))),(int(x1*0.97),(int(CENTER[1]+GA_y))), Line_Color,3)
    cv2.line(frame,(int(CENTER[0]+GA_x),(int(CENTER[1]-GA_y))),(int(x1*0.97),(int(CENTER[1]-GA_y))), Line_Color,3)
    cv2.line(frame,(int(CENTER[0]-GA_x),(int(CENTER[1]+GA_y))),(int(x0*1.03),(int(CENTER[1]+GA_y))), Line_Color,3)
    cv2.line(frame,(int(CENTER[0]-GA_x),(int(CENTER[1]-GA_y))),(int(x0*1.03),(int(CENTER[1]-GA_y))), Line_Color,3)
    #Center Circle
    cv2.circle(frame,(int(CENTER[0]),(int(CENTER[1]))),int(Center_Circle),Line_Color ,3, 8,0)


def init_realtime_display():
    """Prepare empty field for external display updates."""
    global _realtime_initialized, _realtime_field_params, _realtime_empty_field, empty_field
    if _realtime_initialized:
        return _realtime_field_params

    # scale/offset boundary & center (center the field in the frame)
    # Work on a copy to avoid mutating the original reference unexpectedly
    field_pts = list(BOUNDARY)
    in_field_width = abs(field_pts[0][0] - field_pts[1][0])
    in_field_height = abs(field_pts[0][1] - field_pts[7][1])
    offset_x = FRAME_WIDTH // 2 - int((field_pts[0][0] + field_pts[1][0]) / 2)
    offset_y = FRAME_HEIGHT // 2 - int((field_pts[0][1] + field_pts[7][1]) / 2)
    OFFSET[0], OFFSET[1] = offset_x, offset_y

    for index, position in enumerate(field_pts):
        x = int(position[0] * SCALE) + OFFSET[0]
        y = int(position[1] * SCALE) + OFFSET[1]
        # clamp to frame to avoid stray lines
        x = max(1, min(x, FRAME_WIDTH - 2))
        y = max(1, min(y, FRAME_HEIGHT - 2))
        field_pts[index] = (x, y)
    BOUNDARY[:] = field_pts
    center_x = (BOUNDARY[0][0] + BOUNDARY[1][0]) / 2
    center_y = (BOUNDARY[0][1] + BOUNDARY[7][1]) / 2
    CENTER[0], CENTER[1] = int(center_x), int(center_y)

    In_Field_Width = abs(BOUNDARY[0][0] - BOUNDARY[1][0])  
    In_Field_Height = abs(BOUNDARY[0][1] - BOUNDARY[7][1])
    FB_x           = int(In_Field_Width*80/340)
    FB_y           = int(In_Field_Height*40/180)
    PK_x           = int(In_Field_Width*110/340)
    Penalty_y      = int(In_Field_Height*65/180)
    Center_Circle  = int(In_Field_Height*30/180)
    GA_x            = int(In_Field_Width*150/340)
    GA_y            = int(In_Field_Height*45/180)

    _realtime_field_params = {
        "FB_x": FB_x, "FB_y": FB_y, "PK_x": PK_x, "Penalty_y": Penalty_y,
        "Center_Circle": Center_Circle, "GA_x": GA_x, "GA_y": GA_y
    }

    _realtime_empty_field = np.zeros((1080,1920,3), np.uint8)
    _realtime_empty_field = cv2.resize(_realtime_empty_field, (int(FRAME_WIDTH), int(FRAME_HEIGHT)))
    draw_empty(_realtime_empty_field, FB_x, FB_y, PK_x, Penalty_y, Center_Circle, GA_x, GA_y)
    empty_field = _realtime_empty_field  # for collision checks
    try:
        st_A.Initialize(CMD, st_A.Player.pos)
        st_A.strategy_update_field(BOUNDARY, CENTER, PK_x, FB_x, FB_y, Penalty_y, GA_x, GA_y)
    except Exception:
        pass
    _realtime_initialized = True
    return _realtime_field_params


def update_external_state(team_pos=None, team_vec=None, ball=None):
    """Map external camera data into simulator Player/OPPO/ball for rendering."""
    # team_pos: list of positions [[x,y], ...], team_vec: direction vectors
    if not _realtime_initialized:
        init_realtime_display()
    ox, oy = OFFSET[0], OFFSET[1]
    scale = SCALE
    try:
        if team_pos is not None and len(team_pos) > 0 and len(team_pos[0]) >= 2:
            st_A.Player.pos = [float(team_pos[0][0]) * scale + ox, float(team_pos[0][1]) * scale + oy]
        if team_pos is not None and len(team_pos) > 1 and len(team_pos[1]) >= 2:
            st_A.OPPO.pos = [float(team_pos[1][0]) * scale + ox, float(team_pos[1][1]) * scale + oy]
    except Exception:
        pass

    try:
        if team_vec is not None and len(team_vec) > 0 and len(team_vec[0]) >= 2:
            st_A.Player.dir = [float(team_vec[0][0]), float(team_vec[0][1])]
        if team_vec is not None and len(team_vec) > 1 and len(team_vec[1]) >= 2:
            st_A.OPPO.dir = [float(team_vec[1][0]), float(team_vec[1][1])]
    except Exception:
        pass

    global ball_pos
    try:
        if ball is not None and len(ball) >= 2:
            bx = float(ball[0]) * scale + ox
            by = float(ball[1]) * scale + oy
            ball_pos = [int(bx), int(by)]
            st_A.ball_pos = [int(bx), int(by)]
    except Exception:
        pass


def render_realtime_frame():
    """Render one frame of simulator view based on current Player/OPPO/ball."""
    if not _realtime_initialized:
        init_realtime_display()
    params = _realtime_field_params
    frame = _realtime_empty_field.copy()
    draw_object(
        frame,
        params["FB_x"], params["FB_y"], params["PK_x"], params["Penalty_y"],
        params["Center_Circle"], params["GA_x"], params["GA_y"]
    )
    cv2.imshow('simulator view', frame)
    cv2.waitKey(1)


def render_realtime_frame_raw():
    """Return one frame (BGR ndarray) without displaying."""
    if not _realtime_initialized:
        init_realtime_display()
    params = _realtime_field_params
    frame = _realtime_empty_field.copy()
    draw_object(
        frame,
        params["FB_x"], params["FB_y"], params["PK_x"], params["Penalty_y"],
        params["Center_Circle"], params["GA_x"], params["GA_y"]
    )
    return frame


def close_realtime():
    try:
        cv2.destroyWindow('simulator view')
    except cv2.error:
        pass


def _display_available():
    if os.name == "nt":
        return True
    return bool(os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))


def _norm_vec(vec):
    try:
        if vec is None or len(vec) < 2:
            return None
        vx, vy = float(vec[0]), float(vec[1])
    except Exception:
        return None
    mag = math.hypot(vx, vy)
    if mag < 1e-6:
        return None
    return [vx / mag, vy / mag]


def _overlay_simulator_arrows(
    frame_bgr,
    robot_positions,
    robot_dirs,
    attacker_mod,
    offset=None,
    scale=1.0,
    fallback_ball=None,
):
    if attacker_mod is None:
        return frame_bgr
    off_x, off_y = (offset or [0, 0])[0], (offset or [0, 0])[1]

    ready_point = getattr(attacker_mod, "ready_point", None)
    target_vec = getattr(attacker_mod, "target_vec", None)
    if target_vec is None:
        target_vec = getattr(attacker_mod, "target_vec_90", None)
    goal_point = getattr(attacker_mod, "goal_point", None)
    ball_pos_src = getattr(attacker_mod, "ball_pos", None)

    kick_point = None
    if isinstance(ball_pos_src, (list, tuple)) and len(ball_pos_src) >= 2:
        kick_point = [float(ball_pos_src[0]), float(ball_pos_src[1])]
    elif isinstance(fallback_ball, (list, tuple)) and len(fallback_ball) >= 2:
        kick_point = [float(fallback_ball[0]), float(fallback_ball[1])]

    ready_ok = isinstance(ready_point, (list, tuple)) and len(ready_point) >= 2
    if ready_ok:
        try:
            if float(ready_point[0]) == 0.0 and float(ready_point[1]) == 0.0:
                if not (isinstance(ball_pos_src, (list, tuple)) and len(ball_pos_src) >= 2 and (float(ball_pos_src[0]) != 0.0 or float(ball_pos_src[1]) != 0.0)):
                    ready_ok = False
        except Exception:
            ready_ok = False
    if not ready_ok and isinstance(fallback_ball, (list, tuple)) and len(fallback_ball) >= 2:
        ready_point = [float(fallback_ball[0]), float(fallback_ball[1])]
        ready_ok = True

    if not (isinstance(goal_point, (list, tuple)) and len(goal_point) >= 2):
        try:
            boundary_src = ORIG_BOUNDARY if ORIG_BOUNDARY else BOUNDARY
            xs = [pt[0] for pt in boundary_src]
            ys = [pt[1] for pt in boundary_src]
            goal_point = [max(xs), (min(ys) + max(ys)) / 2.0]
        except Exception:
            goal_point = None

    if ready_ok and not (isinstance(target_vec, (list, tuple)) and len(target_vec) >= 2):
        if isinstance(goal_point, (list, tuple)) and len(goal_point) >= 2:
            tv = _norm_vec([float(goal_point[0]) - float(ready_point[0]), float(goal_point[1]) - float(ready_point[1])])
            if tv is not None:
                target_vec = tv

    if isinstance(goal_point, (list, tuple)) and len(goal_point) >= 2:
        try:
            goal_px = (int(float(goal_point[0]) * scale + off_x), int(float(goal_point[1]) * scale + off_y))
            cv2.circle(frame_bgr, goal_px, 6, (0, 250, 250), 2, 8, 0)
            cv2.putText(frame_bgr, "Goal", (goal_px[0] + 6, goal_px[1] - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 250, 250), 2, cv2.LINE_AA)
        except Exception:
            pass

    def _draw_one(pos, dir_vec):
        if not (isinstance(pos, (list, tuple)) and len(pos) >= 2):
            return None, None
        try:
            p_raw = [float(pos[0]), float(pos[1])]
        except Exception:
            return None, None
        f_dir = _norm_vec(dir_vec)
        if f_dir is None:
            return None, None
        r_dir = [f_dir[1], -f_dir[0]]
        start_point = (int(p_raw[0] * scale + off_x), int(p_raw[1] * scale + off_y))
        arrow_len = 50
        end_point = (int(p_raw[0] + f_dir[0] * arrow_len), int(p_raw[1] + f_dir[1] * arrow_len))
        end_point = (int(end_point[0] * scale + off_x), int(end_point[1] * scale + off_y))
        cv2.arrowedLine(frame_bgr, start_point, end_point, (0, 250, 250), 3, tipLength=0.3)

        end_point_right = (int(p_raw[0] + r_dir[0] * arrow_len), int(p_raw[1] + r_dir[1] * arrow_len))
        end_point_right = (int(end_point_right[0] * scale + off_x), int(end_point_right[1] * scale + off_y))
        cv2.arrowedLine(frame_bgr, start_point, end_point_right, (0, 0, 250), 3, tipLength=0.3)

        dist_text = None
        ang_text = None
        if ready_ok:
            ready_point_int = [int(ready_point[0]), int(ready_point[1])]
            ready_point_px = (int(ready_point_int[0] * scale + off_x), int(ready_point_int[1] * scale + off_y))
            cv2.arrowedLine(frame_bgr, start_point, ready_point_px, (250, 250, 250), 3, tipLength=0.3)
            if isinstance(target_vec, (list, tuple)) and len(target_vec) >= 2:
                end_point2 = [int(ready_point_int[0] + 30 * target_vec[0]), int(ready_point_int[1] + 30 * target_vec[1])]
                end_point2_px = (int(end_point2[0] * scale + off_x), int(end_point2[1] * scale + off_y))
                cv2.arrowedLine(frame_bgr, ready_point_px, end_point2_px, (100, 250, 250), 1, tipLength=0.3)
        if kick_point is not None:
            try:
                dist_val = math.hypot(float(kick_point[0]) - p_raw[0], float(kick_point[1]) - p_raw[1])
                dist_text = f"Dist: {dist_val:.1f}"
            except Exception:
                pass
        if f_dir is not None and kick_point is not None and isinstance(goal_point, (list, tuple)) and len(goal_point) >= 2:
            try:
                gvx = float(goal_point[0]) - float(kick_point[0])
                gvy = float(goal_point[1]) - float(kick_point[1])
                g_dir = _norm_vec([gvx, gvy])
                if g_dir is not None:
                    dot = max(-1.0, min(1.0, f_dir[0] * g_dir[0] + f_dir[1] * g_dir[1]))
                    ang = math.degrees(math.acos(dot))
                    ang_text = f"Angle: {ang:.1f} deg"
            except Exception:
                pass
        return dist_text, ang_text

    r0_pos = robot_positions[0] if isinstance(robot_positions, (list, tuple)) and len(robot_positions) > 0 else None
    r1_pos = robot_positions[1] if isinstance(robot_positions, (list, tuple)) and len(robot_positions) > 1 else None
    r0_dir = robot_dirs[0] if isinstance(robot_dirs, (list, tuple)) and len(robot_dirs) > 0 else None
    r1_dir = robot_dirs[1] if isinstance(robot_dirs, (list, tuple)) and len(robot_dirs) > 1 else None

    r0_dist, r0_ang = _draw_one(r0_pos, r0_dir)
    r1_dist, r1_ang = _draw_one(r1_pos, r1_dir)

    def _text_or_dash(txt):
        return txt if txt is not None else "-"

    cv2.putText(frame_bgr, f"R0 {_text_or_dash(r0_dist)}", (10, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (250, 250, 250), 2, cv2.LINE_AA)
    cv2.putText(frame_bgr, f"R0 {_text_or_dash(r0_ang)}", (10, 46), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (250, 250, 250), 2, cv2.LINE_AA)
    cv2.putText(frame_bgr, f"R1 {_text_or_dash(r1_dist)}", (10, 74), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (250, 250, 250), 2, cv2.LINE_AA)
    cv2.putText(frame_bgr, f"R1 {_text_or_dash(r1_ang)}", (10, 98), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (250, 250, 250), 2, cv2.LINE_AA)
    return frame_bgr


def _extract_state_from_provider(provider):
    team, dirs, ball = [], [], []
    if provider is None:
        return team, dirs, ball
    if callable(provider):
        try:
            data = provider()
            if isinstance(data, (list, tuple)) and len(data) == 3:
                return data
        except Exception:
            return team, dirs, ball
    running = getattr(provider, "_running", None)
    if running is False:
        return team, dirs, ball
    state_lock = getattr(provider, "_state_lock", None)
    try:
        if state_lock is not None:
            with state_lock:
                team = [p[:] for p in getattr(provider, "team_pos", [])]
                dirs = [d[:] if isinstance(d, (list, tuple)) else d for d in getattr(provider, "team_degree", [])]
                ball = list(getattr(provider, "ball_center", []))
        else:
            team = [p[:] for p in getattr(provider, "team_pos", [])]
            dirs = [d[:] if isinstance(d, (list, tuple)) else d for d in getattr(provider, "team_degree", [])]
            ball = list(getattr(provider, "ball_center", []))
    except Exception:
        team, dirs, ball = [], [], []
    return team, dirs, ball


def _normalize_state(team, dirs, ball):
    team_list = []
    try:
        for p in team:
            if p is not None and len(p) >= 2:
                team_list.append([float(p[0]), float(p[1])])
    except Exception:
        team_list = []
    dirs_list = []
    try:
        for d in dirs:
            if isinstance(d, (list, tuple)) and len(d) >= 2:
                dirs_list.append([float(d[0]), float(d[1])])
            elif isinstance(d, (int, float)):
                rad = math.radians(float(d))
                dirs_list.append([math.cos(rad), math.sin(rad)])
    except Exception:
        dirs_list = []
    ball_list = []
    try:
        if ball is not None and len(ball) >= 2:
            ball_list = [float(ball[0]), float(ball[1])]
    except Exception:
        ball_list = []
    return team_list, dirs_list, ball_list


def _default_team_positions():
    try:
        init_realtime_display()
        ox, oy = OFFSET[0], OFFSET[1]
        sc = SCALE if SCALE else 1.0
        cx, cy = CENTER[0], CENTER[1]
        defaults_px = [
            [cx + 0, cy + 0],
            [cx - 100, cy - 50],
            [cx - 100, cy + 50],
        ]
        return [[(p[0] - ox) / sc, (p[1] - oy) / sc] for p in defaults_px]
    except Exception:
        return []


def _default_ball_position():
    try:
        init_realtime_display()
        ox, oy = OFFSET[0], OFFSET[1]
        sc = SCALE if SCALE else 1.0
        cx, cy = CENTER[0], CENTER[1]
        return [(cx + 171 - ox) / sc, (cy - oy) / sc]
    except Exception:
        return []


def open_realtime_view(provider=None, overlay_enabled=True, title="Simulator View", strategy_mod=None, tk_parent=None):
    if not _display_available():
        print("[simulator] DISPLAY not available; skip opening simulator window.")
        return
    try:
        import tkinter as tk
    except Exception as err:
        print(f"[simulator] tkinter unavailable: {err}")
        return
    global _rt_win, _rt_running, _rt_overlay_enabled, _rt_canvas, _rt_last_team, _rt_last_dirs, _rt_last_ball
    global _rt_strategy_mod, _rt_provider, _rt_root, _realtime_initialized, _realtime_empty_field

    _rt_overlay_enabled = overlay_enabled
    _rt_strategy_mod = strategy_mod
    _rt_provider = provider

    if _rt_running:
        if _rt_win and _rt_win.winfo_exists():
            try:
                _rt_win.title(title)
            except Exception:
                pass
            _rt_win.lift()
            return
        _rt_running = False

    if _rt_win and _rt_win.winfo_exists():
        _rt_win.lift()
        return

    try:
        _realtime_initialized = False
        _realtime_empty_field = None
    except Exception:
        pass

    _rt_last_team = []
    _rt_last_dirs = []
    _rt_last_ball = []

    created_root = False
    if tk_parent is None:
        _rt_root = tk.Tk()
        _rt_win = _rt_root
        created_root = True
    else:
        _rt_root = None
        _rt_win = tk.Toplevel(tk_parent)

    _rt_win.title(title)
    _rt_win.lift()
    _rt_canvas = tk.Label(_rt_win)
    _rt_canvas.pack()
    _rt_running = True

    def _cleanup():
        global _rt_running, _rt_win, _rt_canvas, _rt_root
        _rt_running = False
        try:
            if _rt_win and _rt_win.winfo_exists():
                _rt_win.destroy()
        except Exception:
            pass
        _rt_win = None
        _rt_canvas = None
        if _rt_root is not None:
            try:
                if _rt_root.winfo_exists():
                    _rt_root.destroy()
            except Exception:
                pass
            _rt_root = None

    _rt_win.protocol("WM_DELETE_WINDOW", _cleanup)

    def _tick():
        global _rt_running, _rt_win, _rt_last_team, _rt_last_dirs, _rt_last_ball, _rt_overlay_enabled
        global _rt_strategy_mod, _rt_provider
        if not _rt_running or not (_rt_win and _rt_win.winfo_exists()):
            _cleanup()
            return
        try:
            team, dirs, ball = _extract_state_from_provider(_rt_provider)
            team_list, dirs_list, ball_list = _normalize_state(team, dirs, ball)

            if team_list:
                _rt_last_team = [p[:] for p in team_list]
            elif _rt_last_team:
                team_list = [p[:] for p in _rt_last_team]
            if dirs_list:
                _rt_last_dirs = [v[:] for v in dirs_list]
            elif _rt_last_dirs:
                dirs_list = [v[:] for v in _rt_last_dirs]
            if ball_list:
                _rt_last_ball = list(ball_list)
            elif _rt_last_ball:
                ball_list = list(_rt_last_ball)

            if team_list and len(dirs_list) < len(team_list):
                for idx in range(len(dirs_list), len(team_list)):
                    if _rt_last_dirs and idx < len(_rt_last_dirs):
                        dirs_list.append(list(_rt_last_dirs[idx]))
                    else:
                        dirs_list.append([1.0, 0.0])
                _rt_last_dirs = [v[:] for v in dirs_list]

            if not team_list and not _rt_last_team:
                team_list = _default_team_positions()
                if team_list:
                    _rt_last_team = [p[:] for p in team_list]
            if not ball_list and not _rt_last_ball:
                ball_list = _default_ball_position()
                if ball_list:
                    _rt_last_ball = list(ball_list)

            update_external_state(team_list, dirs_list, ball_list)

            sim_strategy = _rt_strategy_mod if _rt_overlay_enabled else None
            if sim_strategy is not None:
                if not getattr(sim_strategy, "_sim_field_inited", False):
                    try:
                        sim_strategy.strategy_update_field(1, ORIG_BOUNDARY, ORIG_CENTER, 0, 0, 0, 0, 0, 0)
                        sim_strategy._sim_field_inited = True
                    except Exception:
                        pass
                try:
                    oppo_list = [team_list[1]] if len(team_list) > 1 else []
                    sim_strategy.Update_Robo_Info(dirs_list, team_list, oppo_list, ball_list)
                except Exception:
                    pass

            frame_bgr = render_realtime_frame_raw()
            if sim_strategy is not None:
                frame_bgr = _overlay_simulator_arrows(
                    frame_bgr,
                    team_list,
                    dirs_list,
                    sim_strategy,
                    OFFSET,
                    SCALE,
                    fallback_ball=ball_list,
                )

            frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
            success, buf = cv2.imencode(".png", frame_rgb)
            if success:
                img = None
                try:
                    img_data = base64.b64encode(buf).decode("ascii")
                    img = tk.PhotoImage(master=_rt_win, data=img_data, format="png")
                except Exception as err:
                    print(f"[simulator] tk image decode failed: {err}")
                    img = None
                if img is not None and _rt_canvas is not None:
                    _rt_canvas.configure(image=img)
                    _rt_canvas.image = img
                    _rt_canvas.update_idletasks()
        except Exception as err:
            print(f"[simulator] tick error: {err}")
        finally:
            if _rt_running and _rt_win and _rt_win.winfo_exists():
                try:
                    _rt_win.after(100, _tick)
                except Exception:
                    _cleanup()

    _rt_win.after(0, _tick)
    if created_root:
        _rt_win.mainloop()


def run_snapshot(ball=None, team_pos=None, team_vec=None):
    """
    Start simulator view with a snapshot of ball/robot state.
    Press 's' to start/pause simulation (same hotkey as原本 main).
    """
    global _snapshot_mode, _snapshot_start, _snapshot_initial, ball_pos, CMD
    init_realtime_display()
    _snapshot_mode = True
    _snapshot_start = False

    # 記錄初始狀態，reset 時使用
    def _store_initial():
        nonlocal ball, team_pos, team_vec
        _snapshot_initial["ball"] = list(ball) if ball else None
        _snapshot_initial["team_pos"] = [p[:] for p in team_pos] if team_pos else None
        _snapshot_initial["team_vec"] = [v[:] for v in team_vec] if team_vec else None

    def _apply_snapshot():
        nonlocal ball, team_pos, team_vec
        if not team_pos:
            team_pos = [[CENTER[0], CENTER[1]], [CENTER[0]+50, CENTER[1]]]
        if not team_vec:
            team_vec = [[1.0, 0.0], [-1.0, 0.0]]
        if not ball:
            ball_pos[:] = [CENTER[0], CENTER[1]]
        update_external_state(team_pos, team_vec, ball)
        if hasattr(st_A, "Initialize"):
            st_A.Initialize(CMD, getattr(st_A, "Player", _DummyRobot()).pos)

    _store_initial()
    _apply_snapshot()

    params = _realtime_field_params
    empty_field = _realtime_empty_field
    cv2.namedWindow('simulator view')

    start = True  # auto-start when launched from UI; press 's' to pause/resume
    strategy_timer = 0
    send_timer = 0
    cmd_a = ['N1', 'N1', 'N1']

    while True:
        tStart = time.time()
        frame = empty_field.copy()
        draw_object(frame, params["FB_x"], params["FB_y"], params["PK_x"], params["Penalty_y"], params["Center_Circle"], params["GA_x"], params["GA_y"])

        if start:
            if strategy_timer == 0:
                upd_strategy_position()
                success, cmd_atmp = get_strategy()
                cmd_a = cmd_atmp if success else cmd_a
            if send_timer == 0:
                CMD[0:3] = cmd_a
                return_sent_cmd(CMD, True)
            else:
                return_sent_cmd(cmd_a, False)

            allmovement()
            ball_move()

            strategy_timer = strategy_timer + 1 if strategy_timer < STRATEGY_UP-1 else 0
            send_timer = send_timer + 1 if send_timer < SEND_UP-1 else 0

        cv2.imshow('simulator view', frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('s'):
            start = not start
        elif key == ord('r'):
            start = False
            ball = _snapshot_initial.get("ball")
            team_pos = _snapshot_initial.get("team_pos")
            team_vec = _snapshot_initial.get("team_vec")
            _apply_snapshot()

        while time.time() - tStart < 1/FRAME_FPS:
            pass

    cv2.destroyWindow('simulator view')
    _snapshot_mode = False
    _snapshot_start = False

def draw_object(frame,FB_x, FB_y, PK_x, Penalty_y, Center_Circle, GA_x, GA_y):
    """Draw robot, ball and other info"""
     ##########################################################
    # Draw runtime image,like moving locus or moving target  #
    # Since color of runtime image may affect boundary judge #
    # of moving object, they cannot be placed 'in front of'  #
    # field's object (ex, boundary should never hoverd by    #
    # colors other than red), so we should draw them first.  #
    ##########################################################
    #Draw other information for debug. Ignore this block when you are not using strategy1 and 2
    try:
        pass
        frame = st_A.draw_on_simulator(frame)
    except AttributeError:
        pass

    #mark mouse marker
    cv2.rectangle(frame, (Mouse_Position[0], Mouse_Position[1]), (Mouse_Position[0]+MarkerSize, Mouse_Position[1]+MarkerSize), (255, 255, 255), -1)#將目前鼠標參數位置印出
    #mark mode text
    txt = ''#設定螢幕左側的控制狀態
    if mode == 2:
        txt= mode_txt[mode]+str(Setup_Index)
    elif mode == 3:
        txt = 'test'
    else:
        txt = mode_txt[mode]
    cv2.putText(frame, txt, (0, int(CENTER[1])), cv2.FONT_HERSHEY_SIMPLEX,1, TXT_Color, 5, cv2.LINE_AA)
    #mark Robot name
    # 目前只有兩隻因此編號為0,1
    cv2.putText(frame, Robo_name[0], ( round(st_A.Player.pos[0]) , round(st_A.Player.pos[1]) - 35 ) , cv2.FONT_HERSHEY_SIMPLEX,1, (102, 102, 0), 2, cv2.LINE_AA)
    cv2.putText(frame, Robo_name[1], ( round(st_A.OPPO.pos[0]) , round(st_A.OPPO.pos[1]) - 35 ), cv2.FONT_HERSHEY_SIMPLEX,1, (102, 102, 0), 2, cv2.LINE_AA)
    # 目前只有兩隻機器人 
    # Draw the Robot with rotate angle
    RoboColor = [TeamA_Color, TeamB_Color, TeamC_Color, OppoA_Color, OppoB_Color, OppoC_Color]
    for RoboIndex in range(robo_num): 
        setup = st_A.rob(0)
        if RoboIndex == 0 :
            setup = st_A.Player
        elif RoboIndex == 1 :
            setup = st_A.OPPO
    #上面的程式碼可以確定目前要的機器人是自己還setup是對手(不知道要不要畫對手)
        angle = 0
        cross,dot,robo_angle,err = vector_product(setup.dir[0], setup.dir[1], 1, 0, 1)
        angle = 180 if err == 1 else robo_angle /math.pi * 180
        angle = -1 * angle if cross >= 0 else angle
        rot_rect = (setup.pos, (RoboDepth, RoboWidth), angle)
        box = cv2.boxPoints(rot_rect)
        # np.int0 is deprecated/removed; use explicit int32 cast
        box = box.astype(np.int32)
        cv2.drawContours(frame,[box],0,RoboColor[RoboIndex],-1) 
        global Robo_rotate_rect, Robo_rotate_vert
        Robo_rotate_vert[RoboIndex] = box
        Robo_rotate_rect[RoboIndex] = rot_rect
    #mark Team_Robot direction
    cv2.line(frame, (round(st_A.Player.pos[0]),round(st_A.Player.pos[1])) , (int(st_A.Player.pos[0]+Robo_direction_length*st_A.Player.dir[0]), int(st_A.Player.pos[1]+Robo_direction_length*st_A.Player.dir[1])), Robo_direction_Color,3)
    #mark Oppo_Robot direction
    cv2.line(frame, (round(st_A.OPPO.pos[0]),round(st_A.OPPO.pos[1])) , (int(st_A.OPPO.pos[0]+Robo_direction_length*st_A.OPPO.dir[0]), int(st_A.OPPO.pos[1]+Robo_direction_length*st_A.OPPO.dir[1])), Robo_direction_Color,3)
    #mark ball
    cv2.circle(frame,(st_A.ball_pos[0],st_A.ball_pos[1]),BallHalfSize,Ball_Color,-1, 8,0)

def CV_event(event, x, y, flags, param):
    global ball_pos, ball_speed
    setup = st_A.rob(0)
    if Setup_Index == 0 :
        setup = st_A.Player
    elif Setup_Index == 1 :
        setup = st_A.OPPO
    
    if event == cv2.EVENT_MOUSEMOVE and Cursor_Position_Show:
        print('Cousor position X: ',x,' Y: ',y,' Pixel BGR value: ',empty_field[y][x])  
    if event == cv2.EVENT_LBUTTONDOWN:
        if mode == 2 and flags == cv2.EVENT_FLAG_LBUTTON:#Modify Robo position
            setup.pos = [x,y].copy()
        elif mode == 2 and flags == cv2.EVENT_FLAG_SHIFTKEY + cv2.EVENT_FLAG_LBUTTON :      
            d_x = x - setup.pos[0]
            d_y = y - setup.pos[1]
            try:
                x_comp = d_x/math.hypot(d_x,d_y)
                y_comp = d_y/math.hypot(d_x,d_y)
                setup.dir = [x_comp,y_comp]
            except ZeroDivisionError:
                return
        else:
            #change mouse mark coordinate
            global Mouse_Position
            #Robo_Position[Setup_Index] = [x,y].copy()
            Mouse_Position = [x,y]
            print('Modify mouse position X: ',x,' Y: ',y,' Pixel BGR value: ',empty_field[y][x])
    if event == cv2.EVENT_RBUTTONDOWN:#mouse right click event
        ball_speed = 0
        ball_pos = [x,y]
        st_A.Update_Robo_Info( st_A.Player.dir , st_A.Player.pos , st_A.OPPO.pos , ball_pos )
        # st_B.Update_Robo_Info(Robo_Direction[3:6], Robo_Position[3:6], Robo_Position[0:3], Ball_Position)
        print('Modify Ball position X: ',x,' Y: ',y,' Pixel BGR value: ',empty_field[y][x]) 

def allmovement():
    """Tell all the robot for it's command"""
    for RoboIndex, command in enumerate(CMD):
        robomovement(RoboIndex, command[0])
        # print(command[1])
    pass

'''
Move the robot in the simulator
We give a command to the robot, and the robot need to react to it as in the real world,
for this function, we give the robot id and the command give to the robot, then it judge the legitimacy of the move, 
if the move is legal, move it, if not, step one step back, and blink 
@param RoboIndex    =>  the ID of the robot to control
@param command      =>  the command for the robot
'''
def robomovement(RoboIndex, command):
    #Decide direction
    global ball_speed, ball_direction # get excess to the robot position, direction, ballspeed and direction is for accidentily touching the ball

    # Total: 7
    if command == MOVE.FORWARD.value or command == MOVE.BACKWARD.value or \
        command == MOVE.MOVE_LEFT.value or command == MOVE.MOVE_RIGHT.value or \
            command == MOVE.F_FORWARD.value or command == MOVE.F_MOVE_LEFT.value or \
                command == MOVE.F_MOVE_RIGHT.value:
        robo_walk(RoboIndex, command)
        pass

    # Total: 4
    elif command == MOVE.TRUN_LEFT.value or command == MOVE.TRUN_RIGHT.value or \
        command == MOVE.F_TRUN_LEFT.value or command == MOVE.F_TRUN_RIGHT.value:
        # print('我要轉了')
        robo_turn(RoboIndex, command)
        pass
    
    # Total: 2
    elif command == MOVE.RFSHOOT.value or command == MOVE.LFSHOOT.value:
        robo_shoot(RoboIndex, 0)
        pass

    # Total: 2
    elif command == MOVE.RSSHOOT.value or command == MOVE.LSSHOOT.value :
        robo_shoot(RoboIndex, 1)
        pass

    # Total: 2
    elif command == MOVE.RBSHOOT.value or command == MOVE.LBSHOOT.value:
        robo_shoot(RoboIndex, 2)
        pass

    # Total: 2
    elif command == MOVE.LPASS.value or command == MOVE.RPASS.value:
        robo_pass(RoboIndex)
        pass

    # Total: 5
    elif command == MOVE.NONE.value or command == MOVE.DEFENSE.value or \
        command == MOVE.STAND.value or command == MOVE.STOP.value or \
            command == MOVE.REST.value  or command == MOVE.START.value  or \
                command == MOVE.PK_MODE.value:
        pass

    # Total: 3
    elif command == MOVE.F_DEFENSE.value or command == MOVE.L_DEFENSE.value or \
        command == MOVE.R_DEFENSE.value:
        pass
    else:
        print("False command, error")

def robo_walk(RoboIndex, command):
    """Move one step"""
    #Decide direction
    global ball_speed, ball_direction

    d_robo_x,d_robo_y = 0.0,0.0

    distance, unit_x, unit_y = 0.0, 0.0, 0.0
    
    setup = st_A.rob(0)
    if RoboIndex == 0 :
        setup = st_A.Player
    elif RoboIndex == 1 :
        setup = st_A.OPPO
    
    if command == MOVE.F_FORWARD.value:
        distance = Robo_move[0]
        unit_x = setup.dir[0]
        unit_y = setup.dir[1]
    elif command == MOVE.FORWARD.value:
        distance = Robo_move[1]
        unit_x = setup.dir[0]
        unit_y = setup.dir[1]
    elif command == MOVE.BACKWARD.value:        
        distance = Robo_move[2]
        unit_x = -setup.dir[0]
        unit_y = -setup.dir[1]
    elif command == MOVE.MOVE_LEFT.value:         
        distance = Robo_move[3]
        unit_x = setup.dir[1]
        unit_y = -setup.dir[0]
    elif command == MOVE.MOVE_RIGHT.value:        
        distance = Robo_move[4]
        unit_x = -setup.dir[1]
        unit_y = setup.dir[0]
    elif command == MOVE.F_MOVE_LEFT.value:         
        distance = Robo_move[5]
        unit_x = setup.dir[1]
        unit_y = -setup.dir[0]
    elif command == MOVE.F_MOVE_RIGHT.value:        
        distance = Robo_move[6]
        unit_x = -setup.dir[1]
        unit_y = setup.dir[0]
    else:
        pass

    for i in range(0,round(distance),1):   
        d_robo_x = d_robo_x + unit_x
        d_robo_y = d_robo_y + unit_y
        #When moving, check if bump into other robot or boundary or ball
        bump_boundary = 0
        #Calculate new rectangle of next move (in OpenCV, rotated rectangle angle is in degree not radius)
        tmp_angle = 0
        tmp_cross, tmp_dot, robo_angle, tmp_err = vector_product(setup.dir[0], setup.dir[1], 1, 0, 1)
        tmp_angle = 180 if tmp_err == 1 else robo_angle /math.pi * 180
        tmp_angle = -1 * tmp_angle if tmp_cross >= 0 else tmp_angle
        tmp_rot_rect = ((setup.pos[0]+d_robo_x, setup.pos[1]+d_robo_y), (RoboDepth, RoboWidth), tmp_angle)
        #Parse this new rectangle to bump detection
        bump_boundary,__ = robo_bump(RoboIndex, tmp_rot_rect)
        #If bump with ball then ball moving
        if bump_boundary == 3:
            ball_speed = 3*Ball_bump_speed
            ball_direction = [unit_x, unit_y]
        #Below should be the final case since it might break the loop
        #If robot do bump something
        if bump_boundary != 0 :
            # d_robo_x = d_robo_x - unit_x
            # d_robo_y = d_robo_y - unit_y
            d_robo_x = 0
            d_robo_y = 0
            break
        
        
    setup.pos[0] = setup.pos[0] + d_robo_x
    setup.pos[1] = setup.pos[1] + d_robo_y

def robo_turn(RoboIndex, command):#direction 1:counterclockwise  0:clockwise
    """Turn Robo one time"""
    global ball_speed, ball_direction

    setup = st_A.rob(0)
    if RoboIndex == 0 :
        setup = st_A.Player
    elif RoboIndex == 1 :
        setup = st_A.OPPO
    
    x_dir = setup.dir[0]
    y_dir = setup.dir[1]

    rotate_angle = 0
    if command == MOVE.TRUN_RIGHT.value:
        rotate_angle = Robo_rotate_speed
    elif command == MOVE.TRUN_LEFT.value:
        rotate_angle = -1 * Robo_rotate_speed
    elif command == MOVE.F_TRUN_RIGHT.value:
        rotate_angle = Robo_rotate_speed*2
    elif command == MOVE.F_TRUN_LEFT.value:
        rotate_angle = -2 * Robo_rotate_speed
    rotate_cos = math.cos(rotate_angle)
    rotate_sin = math.sin(rotate_angle)
    x_final, y_final = x_dir*rotate_cos - y_dir*rotate_sin, x_dir*rotate_sin + y_dir*rotate_cos
    
    #Collision might happen when rotate. Just as what we do in Robo moving, Calculate new rectangle of next move first.
    tmp_angle = 0
    tmp_cross, tmp_dot, robo_angle, tmp_err = vector_product(x_final, y_final, 1, 0, 1)
    tmp_angle = 180 if tmp_err == 1 else robo_angle /math.pi * 180
    tmp_angle = -1 * tmp_angle if tmp_cross >= 0 else tmp_angle
    tmp_rot_rect = ((setup.pos[0], setup.pos[1]), (RoboDepth, RoboWidth), tmp_angle)
    #Parse this new rectangle to bump detection
    bump_boundary,rot_bump_position = robo_bump(RoboIndex, tmp_rot_rect)
    if bump_boundary == 3:
        ball_speed = Ball_rotbump_speed
        ball_dir_x = ball_pos[0] - rot_bump_position[0]
        ball_dir_y = ball_pos[1] - rot_bump_position[1]
        length = math.hypot(ball_dir_x, ball_dir_y)
        ball_d_x = ball_dir_x/length
        ball_d_y = ball_dir_y/length
        ball_direction = [ball_d_x, ball_d_y]
    elif bump_boundary == 0:
        setup.dir = [x_final, y_final]

def robo_bump(RoboIndex, rot_rect):
    """Detect bumping with Boundary, ball,Robot when robot is moving"""
    bump_boundary = 0
    rot_bump_position = [0, 0]
    vertices = cv2.boxPoints(rot_rect)
    vertices = np.int0(vertices).tolist()
    ###################################################################################
    # Detect Robot-Robot collision by OpenCV rotatedRectangleIntersection function    #
    # Return 1 if bump                                                                #
    ###################################################################################
    if bump_boundary == 0:
        for rec_index, rect in enumerate(Robo_rotate_rect):
            if rec_index == RoboIndex:
                pass
            else:
                intersect = cv2.rotatedRectangleIntersection(rect, rot_rect)
                if intersect[0] != 0:
                    bump_boundary = 1
                    print(RoboIndex,' bump with ',rec_index)
                    break
    ###################################################################################
    # Detect Robot-boundary collision by reading rectangle vertices' BGR value        #
    # Return 2 if bump                                                                #
    ###################################################################################
    if bump_boundary == 0:
        for vert_index, vert in enumerate(vertices):
            if (frame[vert[1]][vert[0]] == Boundary_Color).all():
                bump_boundary = 2
                break
    ###################################################################################
    # Detect Robot-ball collision by vector (use vector to calculate distance)        #
    # Return 3 if bump                                                                #
    # To calculate point P to line AB, consider vector AC,which is projection of      #
    # vector AP on Vector AB. AC has same direction asˊ AB and its scale can be        #
    # obtained by [(AP*AB)/(AB*AB)]. If scale <1 then C is located between AB, length #
    # is equal to length of vector CP needless to say. If scale <=0 then length is    #
    # equal to length AP since AP has a opposite projection on AB. If scale >=1 then  #
    # length is equal to lengthBP clearly.                                            #
    ###################################################################################
    if bump_boundary == 0:
        for vert_index, vert in enumerate(vertices):
            line_pointA = [vert[0], vert[1]]
            end_index = 0 if vert_index == 3 else vert_index + 1
            line_pointB = [vertices[end_index][0], vertices[end_index][1]]
            vect_A_B = [line_pointB[0]-line_pointA[0], line_pointB[1]-line_pointA[1]]#Vector A2B
            vect_A_ball = [ball_pos[0] - line_pointA[0], ball_pos[1] - line_pointA[1]]#Vector A2Ball
            #Calculate the Projection of A2Ball on A2B i.e  (AdotB)/lengthAB^2 * vectorAB
            dot = vector_product(vect_A_ball[0],vect_A_ball[1],vect_A_B[0],vect_A_B[1],0)[1]
            ABlength_sqa = math.pow(math.hypot(vect_A_B[0], vect_A_B[1]),2)
            Projection_scale = dot/ABlength_sqa
            BallonEgde_x, BallonEgde_y = vect_A_B[0]*Projection_scale, vect_A_B[1]*Projection_scale
            vect_BallonEgde = [BallonEgde_x, BallonEgde_y]
            #Calculate ball to edge distance
            edge_ball_distance = 0
            if Projection_scale <= 0:
                rot_bump_position = line_pointA
                edge_ball_distance = math.hypot(vect_A_ball[0],vect_A_ball[1])
            elif Projection_scale >= 1:
                rot_bump_position = line_pointB
                vect_B_ball = [x1 - x2 for (x1, x2) in zip(vect_A_ball, vect_A_B)]
                edge_ball_distance = math.hypot(vect_B_ball[0], vect_B_ball[1])
            else:
                rot_bump_position = [x1 + x2 for (x1, x2) in zip(line_pointA, vect_BallonEgde)]
                vect_P_ball = [x1 - x2 for (x1, x2) in zip(vect_A_ball, vect_BallonEgde)]#Projection point to Ball
                edge_ball_distance = math.hypot(vect_P_ball[0], vect_P_ball[1])
            #Judge if bump with ball
            if edge_ball_distance <= 6:
                bump_boundary = 3
                print(RoboIndex,' bump ball --distance: ',edge_ball_distance)
                break
    return bump_boundary ,rot_bump_position #Second return only meaningful when bump_boundary == 3

def vector_product(p1_x, p1_y, p2_x, p2_y, unit_vector=0):
    """
    
    輸入兩項量，回傳值為外積、內積、角度、錯誤訊息()
    
    Description:
        CAlculate cross/dot product and angle of two given vector
    Parameters:
        p1, p2 are two endpoints
        unit_vector: set to 1 if given vectors are unit vector
    Return:
        cross product, dor product, angle, and error flag, 1 if there is error.
    """
    ValueErrorFlag = 0
    cro = p1_x * p2_y - p1_y * p2_x
    dot = p1_x * p2_x + p1_y * p2_y
    dot_angle = dot if unit_vector == 1 else dot/(math.hypot(p1_x,p1_y)*math.hypot(p2_x,p2_y))
    angle = 0
    try:
        angle = math.acos(dot_angle)
    except  ValueError:
        ValueErrorFlag = 1
        return cro,dot,0,ValueErrorFlag
    return cro,dot,angle,ValueErrorFlag

def robo_shoot(RoboIndex, cmd = 0):

    global ball_speed, ball_direction

    setup = st_A.rob(0)
    if RoboIndex == 0 :
        setup = st_A.Player
    elif RoboIndex == 1 :
        setup = st_A.OPPO
    
    d_ball_x = ball_pos[0]-setup.pos[0]
    d_ball_y = ball_pos[1]-setup.pos[1]
    distance = math.hypot(d_ball_x, d_ball_y)
    unit_ball_x = d_ball_x/distance
    unit_ball_y = d_ball_y/distance
    cross,dot,ball_angle_dif,err = vector_product(setup.dir[0], setup.dir[1], unit_ball_x, unit_ball_y,1)
    ball_angle_dif = math.pi if err and dot <= -1 else ball_angle_dif
    if ball_angle_dif > Robo_kick_angle_threshold and cmd == 0:
        print(RoboIndex,' kick angle too large, miss')
        return
    elif (ball_angle_dif - 0.5 * math.pi) > Robo_kick_angle_threshold and cmd == 1:
        print(RoboIndex,' kick angle too large, miss')
        return
    elif (ball_angle_dif - 1 * math.pi) > Robo_kick_angle_threshold and cmd == 2:
        print(RoboIndex,' kick angle too large, miss')
        return
    else:
        #If ball is not too far
        if distance <= Kickable_distance:
            if cmd == 0:
                ball_direction = [setup.dir[0],setup.dir[1]]
                ball_speed = Ball_kicked_speed
            elif cmd == 1:
                ball_direction = [unit_ball_x,unit_ball_y]
                ball_speed = Ball_kicked_speed * 1  # change
            elif cmd == 2:
                ball_direction = [-setup.dir[0],-setup.dir[1]]
                ball_speed = Ball_kicked_speed
            else:
                pass
            
            print('Robo ',RoboIndex,' Play a shoot')   
        else:
            print(RoboIndex,' too far, miss')
            return

def robo_pass(RoboIndex):
    """Robot Pass"""
    global ball_speed, ball_direction

    setup = st_A.rob(0)
    if RoboIndex == 0 :
        setup = st_A.Player
    elif RoboIndex == 1 :
        setup = st_A.OPPO
    
    #Angle is correct then kick
    d_ball_x = ball_pos[0]-setup.pos[0]
    d_ball_y = ball_pos[1]-setup.pos[1]
    distance = math.hypot(d_ball_x, d_ball_y)
    unit_ball_x = d_ball_x/distance
    unit_ball_y = d_ball_y/distance
    cross,dot,ball_angle_dif,err = vector_product(setup.dir[0], setup.dir[1], unit_ball_x, unit_ball_y,1)
    ball_angle_dif = math.pi if err and dot <= -1 else ball_angle_dif
    if ball_angle_dif > Robo_kick_angle_threshold:
        print(RoboIndex,' PASS angle too large, miss')
        return
    else:
        #If ball is not too far
        if distance <= Kickable_distance:
            ball_direction = [setup.dir[0],setup.dir[1]]
            ball_speed = Ball_passing_speed
            print('Robo ',RoboIndex,' Play a PASS')   
        else:
            print(RoboIndex,'PASS too far, miss')
            return

def ball_move():
    """Simulation of ball"""
    global ball_speed,ball_direction
    try:
        d_ball_X, d_ball_Y = 0, 0
        Color = [Boundary_Color,TeamA_Color,TeamB_Color,TeamC_Color,OppoA_Color,OppoB_Color,OppoC_Color]
        for i in range(0,int(ball_speed),1):
            bound_x, bound_y = 0, 0

            #Check if collision happen when moving to next position
            #X and Y are separated
            d_ball_X = d_ball_X + 1 * ball_direction[0]
            size_offset = BallHalfSize if d_ball_X >= 0 else -1 * BallHalfSize
            for bound_color in Color:
                if (empty_field[ int(round(ball_pos[1]+d_ball_Y+size_offset)) ][ int(round(ball_pos[0]+d_ball_X+size_offset)) ] == bound_color).all() or\
                    (empty_field[ int(round(ball_pos[1]+d_ball_Y-size_offset)) ][ int(round(ball_pos[0]+d_ball_X+size_offset)) ] == bound_color).all():
                    bound_x = 1
                    break
            d_ball_X = d_ball_X - 1 * ball_direction[0]

            d_ball_Y = d_ball_Y + 1 * ball_direction[1]
            size_offset = BallHalfSize if d_ball_Y >= 0 else -1 * BallHalfSize
            for bound_color in Color:
                if (empty_field[ int(round(ball_pos[1]+d_ball_Y+size_offset)) ][ int(round(ball_pos[0]+d_ball_X+size_offset)) ] == bound_color).all() or\
                    (empty_field[ int(round(ball_pos[1]+d_ball_Y+size_offset)) ][ int(round(ball_pos[0]+d_ball_X-size_offset)) ] == bound_color).all():
                    bound_y = 1
                    break
            d_ball_Y = d_ball_Y - 1 * ball_direction[1]
            
            d_ball_X = d_ball_X  if bound_x else d_ball_X + ball_direction[0]
            d_ball_Y = d_ball_Y  if bound_y else d_ball_Y + ball_direction[1]
            ball_direction[0] = -1 * ball_direction[0] if bound_x else ball_direction[0]
            ball_direction[1] = -1 * ball_direction[1] if bound_y else ball_direction[1]

    except ZeroDivisionError:
        d_ball_X, d_ball_Y= 0, 0
    finally:
        ball_pos[0] = ball_pos[0] + int(round(d_ball_X))
        ball_pos[1] = ball_pos[1] + int(round(d_ball_Y))
        ball_speed = 0 if ball_speed - Ball_n_acce < 0 else ball_speed - Ball_n_acce

def get_strategy():
    """Simulator get strategy"""
    try:
        cmd_a = st_A.strategy()
        #cmd_b = st_B.strategy()
    except AttributeError:
        return 0,0
    print('CMD2',cmd_a)
    return 1, cmd_a

def parse_strategy_field(PK_x, FB_x, FB_y, Penalty_y):
    """Parse field parameter to strategy"""
    try:
        (sideA, sideB) = (1, -1) if Side == 1 else (-1, 1)
        st_A.strategy_update_field(sideA, BOUNDARY, CENTER, PK_x, FB_x, FB_y, Penalty_y, GA_x, GA_y)
        # st_B.strategy_update_field(sideB, BOUNDARY, CENTER, PK_x, FB_x, FB_y, Penalty_y, GA_x, GA_y)
    except AttributeError:
        pass

def return_sent_cmd(send_data, sent):
    """Pass sent command to strategy"""
    try:
        st_A.get_sent_cmd(send_data[0:3], sent)
        # st_B.get_sent_cmd(send_data[3:6], sent)
    except AttributeError:
        pass

def upd_strategy_position():
    """Simulator Opject position update"""

    
    st_A.Update_Robo_Info( st_A.Player.dir , st_A.Player.pos , st_A.OPPO.pos , ball_pos )

def reset_obj(Robo_start_x,Robo_start_y,Keeper_start_x):
    st_A.ball_pos = [int(CENTER[0] + Robo_start_x ), int(CENTER[1])]
    #Player
    st_A.Player.pos = [float(CENTER[0] ),float(CENTER[1])]
    #OPPO
    st_A.OPPO.pos = [float(CENTER[0] + Keeper_start_x),float(CENTER[1])]
    print(round(st_A.Player.pos[0],2),round(st_A.Player.pos[1],2))


if __name__ == '__main__':

    for index,position in enumerate(BOUNDARY):
        BOUNDARY[index] = (int(position[0]*SCALE)+OFFSET[0],int(position[1]*SCALE)+OFFSET[1])
    center_x = (BOUNDARY[0][0] + BOUNDARY[1][0])/2
    center_y = (BOUNDARY[0][1] + BOUNDARY[7][1])/2
    CENTER = [int(center_x),int(center_y)]
    print('Virtual Field Center Position',CENTER)

    # Calculate field building parameter
    In_Field_Width = abs(BOUNDARY[0][0] - BOUNDARY[1][0])  
   
    In_Field_Height = abs(BOUNDARY[0][1] - BOUNDARY[7][1])
    FB_x           = int(In_Field_Width*80/340)
    FB_y           = int(In_Field_Height*40/180)
    PK_x           = int(In_Field_Width*110/340)
    Penalty_y      = int(In_Field_Height*65/180)
    Center_Circle  = int(In_Field_Height*30/180)
    Robo_start_x   = int(In_Field_Width*80/340)
    Robo_start_y   = int(In_Field_Height*40/180)
    Keeper_start_x = int(In_Field_Width*160/340)
    GA_x            = int(In_Field_Width*150/340)
    GA_y            = int(In_Field_Height*45/180)
    
    cv2.namedWindow('my image')
    cv2.setMouseCallback('my image',CV_event)#228行
    # Draw empty field
    empty_field = np.zeros((1080,1920,3), np.uint8)
    empty_field = cv2.resize(empty_field, ( int(FRAME_WIDTH) , int(FRAME_HEIGHT) ))
    
    draw_empty(empty_field,FB_x,FB_y,PK_x,Penalty_y,Center_Circle, GA_x, GA_y)
    st_A.Initialize(CMD,st_A.Player.pos)
    # draw_object(empty_field,FB_x, FB_y, PK_x, Penalty_y, Center_Circle, GA_x, GA_y)
    
    reset_obj(Robo_start_x,Robo_start_y,Keeper_start_x)
    #Parse field parameter to strategy and initialize strategy
    st_A.strategy_update_field(BOUNDARY, CENTER, PK_x, FB_x, FB_y, Penalty_y, GA_x, GA_y) #更正後

    cmd_a, cmd_b = [None]*3, [None]*3

    start = 0 #start after enter 's'
    strategy_timer, send_timer = 0, 0
    
    while True:
        # Start timer
        tStart = time.time()

        # copy empty field and draw object
        frame = empty_field.copy()
        draw_object(frame,FB_x,FB_y,PK_x,Penalty_y,Center_Circle, GA_x, GA_y)
        time.sleep(0.05)

        #If ball is in the goal area then this match is finished.
        if st_A.ball_pos[0] <= BOUNDARY[0][0] or st_A.ball_pos[0] >= BOUNDARY[1][0]\
                or st_A.ball_pos[1] <= BOUNDARY[0][1] or st_A.ball_pos[1] >= BOUNDARY[7][1]:
            start, mode = 0, 1
            print(st_A.ball_pos)

        
        if start:
        ###################################### strategy ##########################################
            print('strategy_timer',strategy_timer)
            print('send_timer',send_timer)
            if strategy_timer == 0:
                # Update strategy information first and then get strategy
                upd_strategy_position()
                # Get stategy
                success,cmd_atmp = get_strategy()
                cmd_a = cmd_atmp if success else cmd_a
                print('CMD3',cmd_a)
                print('send_timer = ',send_timer)

            if send_timer == 0:
                # Send(simulate) command
                CMD[0:3] = cmd_a
                # Robo_command[3:6] = cmd_b
                return_sent_cmd(CMD, True)
            else:
                return_sent_cmd(cmd_a, False)

            print('CMD4',CMD)
            allmovement()
            ball_move()

            # Update timer#延遲部分
            # strategy_timer = strategy_timer + 1 if strategy_timer < STRATEGY_UP-1 else 0
            # send_timer = send_timer + 1 if send_timer < SEND_UP-1 else 0
#
        # Show the result of this loop
        # print(ball_pos[0],ball_pos[1])
        cv2.imshow('my image', frame )
        
        ################################## Keyboard detector #####################################
        k = cv2.waitKey(1) & 0xFF
        if  k == ord('q'):# Quit
            break
        elif  k == ord('s'):# Start
            start, mode = 1, 0
        elif  k == ord('p'):# Pause
            start, mode = 0, 1
            
        elif  k == ord('r'):# Reset Simulation
            start, mode = 0, 1
            reset_obj(Robo_start_x,Robo_start_y,Keeper_start_x)
        elif  k == ord('c'):# Open Cursor Tracker
            Cursor_Position_Show = not Cursor_Position_Show
            print('Cursor','On' if Cursor_Position_Show else 'OFF')
        elif  k == ord('k'):# Kick all robot outside the field
            start, mode = 0, 1
            st_A.Player.pos = [ 0.0 , 0.0 ]
            st_A.OPPO.pos = [ 0.0 , 0.0 ]
        elif  k == ord('o'): # Set robot position
            start, mode = 0, 2
        #test
        elif  k == ord('t'): # test
            mode = 3
        #test
        elif  mode == 3 : 
            if k == ord('a'):
                CMD = ['a1','N1','N1']
                print("ball_pos = ",ball_pos)
                allmovement()
                ball_move()
                print('Player.pos = ',round(st_A.Player.pos[0],3),round(st_A.Player.pos[1],3))
                print('Player.dir = ',st_A.Player.dir)
            elif k == ord('d'):
                CMD = ['d1','N1','N1']
                print("ball_pos = ",ball_pos)
                allmovement()
                ball_move()
                print('Player.pos = ',round(st_A.Player.pos[0],3),round(st_A.Player.pos[1],3))
                print('Player.dir = ',st_A.Player.dir)
            elif k == ord('n'):
                CMD = ['q1','N1','N1']#向左轉 
                print("ball_pos = ",ball_pos)
                allmovement()
                ball_move()
                print('Player.pos = ',round(st_A.Player.pos[0],3),round(st_A.Player.pos[1],3))
                print('Player.dir = ',st_A.Player.dir)
            elif k == ord('m'):
                CMD = ['e1','N1','N1']#向右轉 
                print("ball_pos = ",ball_pos)
                allmovement()
                ball_move()
                print('Player.pos = ',round(st_A.Player.pos[0],3),round(st_A.Player.pos[1],3))
                print('Player.dir = ',st_A.Player.dir)

        elif  mode == 2 and k <= ord('5') and k >= ord('0'):
            Setup_Index = k-ord('0') #Robo 0 to 5 index 0 to 5

        while time.time() - tStart < 1/FRAME_FPS:
            pass

    # Close all frames #
    cv2.destroyAllWindows()
