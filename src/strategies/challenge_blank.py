# Minimal blank strategy scaffold for integration/testing.

# Field parameters (updated via strategy_update_field)
BOUNDARY = []
CENTER = [0, 0]
PK = FB = PENALTY_AREA = GOAL_AREA = [0, 0]

# Latest sensor/state snapshots
_angle = []
_location = []
_oppo = []
_ball = []
_count = 0


def strategy_update_field(side, boundary, center, pk_x, fb_x, fb_y, penalty_y, ga_x, ga_y):
    """Receive field information (called once by simulator/main)."""
    global BOUNDARY, CENTER, PK, FB, PENALTY_AREA, GOAL_AREA
    try:
        BOUNDARY = list(boundary) if boundary is not None else []
    except Exception:
        BOUNDARY = []
    try:
        # 防止傳入可呼叫物件或非可迭代型態
        if callable(center):
            CENTER = [0, 0]
        else:
            CENTER = list(center) if center is not None else [0, 0]
    except Exception:
        CENTER = [0, 0]
    PK = [pk_x, 0]
    FB = [fb_x, fb_y]
    PENALTY_AREA = [pk_x, penalty_y]
    GOAL_AREA = [ga_x, ga_y]


def Initialize(cmd=None, pos=None):
    """Initialize internal state; cmd/pos are kept for API compatibility."""
    return True


def Update_Robo_Info(angle, location, oppo, ball, count=0):
    """Update latest sensor data."""
    global _angle, _location, _oppo, _ball, _count
    _angle = angle or []
    _location = location or []
    _oppo = oppo or []
    _ball = ball or []
    _count = count


def strategy():
    """Return a triple-command list; default no-op."""
    return ['N1', 'N1', 'N1']


def get_sent_cmd(cmd, sent):
    """Receive feedback of sent commands (optional)."""
    return
