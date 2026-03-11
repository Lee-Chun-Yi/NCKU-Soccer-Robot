import math


def _norm_angle_deg(deg: float) -> float:
    """Normalize angle to [0, 360)."""
    return (deg % 360.0 + 360.0) % 360.0


def _circular_mean_deg(angles):
    """Average angles on a circle."""
    if not angles:
        return None
    sin_sum = sum(math.sin(math.radians(a)) for a in angles)
    cos_sum = sum(math.cos(math.radians(a)) for a in angles)
    if sin_sum == 0 and cos_sum == 0:
        return _norm_angle_deg(angles[0])
    return _norm_angle_deg(math.degrees(math.atan2(sin_sum, cos_sum)))


def _weighted_mean_angle_deg(left_deg: float, right_deg: float, w_left: float, w_right: float):
    """Weighted circular mean for two angles in degrees."""
    total = max(float(w_left) + float(w_right), 1e-9)
    wl = float(w_left) / total
    wr = float(w_right) / total
    l_rad = math.radians(float(left_deg))
    r_rad = math.radians(float(right_deg))
    x = wl * math.cos(l_rad) + wr * math.cos(r_rad)
    y = wl * math.sin(l_rad) + wr * math.sin(r_rad)
    if abs(x) < 1e-9 and abs(y) < 1e-9:
        return _norm_angle_deg(float(left_deg))
    return _norm_angle_deg(math.degrees(math.atan2(y, x)))


def _prune_samples(samples, now_ts: float, window_sec: float):
    """Remove samples older than window_sec."""
    while samples and now_ts - samples[0][0] > window_sec:
        samples.popleft()


def _mode_angle_from_samples(samples, bin_size_deg: float):
    """Return mode angle from timestamped samples using circular bins."""
    if not samples:
        return None
    num_bins = max(1, int(round(360.0 / bin_size_deg)))
    counts = {}
    bin_members = {}
    half_bin = bin_size_deg / 2.0
    for _, ang in samples:
        ang_norm = _norm_angle_deg(ang)
        bin_idx = int((ang_norm + half_bin) // bin_size_deg) % num_bins
        counts[bin_idx] = counts.get(bin_idx, 0) + 1
        bin_members.setdefault(bin_idx, []).append(ang_norm)
    best_idx = max(counts, key=lambda k: counts[k])
    return _circular_mean_deg(bin_members.get(best_idx, []))


def _push_angle_sample(robot, cam_role: str, angle_deg: float, ts: float):
    """Append angle to robot's per-camera buffer."""
    if cam_role == "Cam1":
        robot.angle_samples_left.append((ts, _norm_angle_deg(angle_deg)))
    else:
        robot.angle_samples_right.append((ts, _norm_angle_deg(angle_deg)))


def _get_stable_angle(robot, ts: float, window_sec: float, bin_size_deg: float):
    """Compute stable angle from both cameras within time window."""
    _prune_samples(robot.angle_samples_left, ts, window_sec)
    _prune_samples(robot.angle_samples_right, ts, window_sec)
    combined = list(robot.angle_samples_left) + list(robot.angle_samples_right)
    if not combined:
        return None
    return _mode_angle_from_samples(combined, bin_size_deg)


def _angular_diff(a, b):
    """Return smallest signed diff between angles in degrees."""
    return ((a - b + 180.0) % 360.0) - 180.0


def _blend_angle(prev_deg, curr_deg, alpha=0.1):
    """Weighted average for angles with 360 wrapping."""
    if prev_deg is None:
        return _norm_angle_deg(curr_deg)
    prev_rad = math.radians(prev_deg)
    curr_rad = math.radians(curr_deg)
    x = (1 - alpha) * math.cos(prev_rad) + alpha * math.cos(curr_rad)
    y = (1 - alpha) * math.sin(prev_rad) + alpha * math.sin(curr_rad)
    return _norm_angle_deg(math.degrees(math.atan2(y, x)))
