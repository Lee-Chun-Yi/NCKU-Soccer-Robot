import math

import cv2
import numpy as np


def _normalize_angle(deg):
    return (deg + 360.0) % 360.0


def _rot_x(deg):
    rad = math.radians(deg)
    c, s = math.cos(rad), math.sin(rad)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]], dtype=np.float32)


def _rot_y(deg):
    rad = math.radians(deg)
    c, s = math.cos(rad), math.sin(rad)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]], dtype=np.float32)


def _rot_z(deg):
    rad = math.radians(deg)
    c, s = math.cos(rad), math.sin(rad)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], dtype=np.float32)


def _compute_top_pose_from_align(R_cam_face, tvec, obj_pts, R_align):
    """
    Map any face pose to a top-face reference pose.
    Returns (R_top, tvec_top, side_len, z_local, cos_theta).
    """
    corners_cam = []
    for p in obj_pts:
        pc = (R_cam_face @ p.reshape(3, 1) + tvec).reshape(3)
        corners_cam.append(pc)
    u = corners_cam[1] - corners_cam[0]
    v = corners_cam[3] - corners_cam[0]
    x_local = u / max(np.linalg.norm(u), 1e-6)
    y_local = v / max(np.linalg.norm(v), 1e-6)
    z_local = np.cross(x_local, y_local)
    z_norm = np.linalg.norm(z_local)
    if z_norm < 1e-6:
        z_local = R_cam_face[:, 2].reshape(3)
    else:
        z_local = z_local / z_norm

    cam_z = np.array([0.0, 0.0, 1.0], dtype=np.float32)
    cos_theta = abs(float(np.dot(z_local, cam_z)))
    edges = [
        np.linalg.norm(corners_cam[1] - corners_cam[0]),
        np.linalg.norm(corners_cam[2] - corners_cam[1]),
        np.linalg.norm(corners_cam[3] - corners_cam[2]),
        np.linalg.norm(corners_cam[0] - corners_cam[3]),
    ]
    side_len = float(np.mean(edges)) / max(cos_theta, 0.1)

    p_center = sum(corners_cam) / 4.0
    t_center = p_center + (-z_local) * (side_len / 2.0)
    R_face_local = np.column_stack((x_local, y_local, z_local)).astype(np.float32)
    R_top = R_face_local @ np.asarray(R_align, dtype=np.float32)

    n_top_cam = R_top @ np.array([0.0, 0.0, 1.0], dtype=np.float32)
    t_top = t_center + n_top_cam * (side_len / 2.0)
    return R_top, t_top.reshape(3, 1), side_len, z_local, cos_theta


def _map_tvec_to_field(tvec, cam_role, config_cam1, config_cam2, marker_id):
    del marker_id
    z_cam = tvec[2][0]
    x_cam = tvec[1][0]
    y_cam = tvec[0][0]
    if cam_role == "Cam1":
        h = config_cam1["camera_height"]
        h_robot = config_cam1["robot_height"]
        val = (x_cam ** 2 + z_cam ** 2) - (h - h_robot) ** 2
        if val < 0:
            return None, None
        x_raw = math.sqrt(val)
        y_raw = y_cam
        xa = (x_raw - 1) * 100
        ya = (y_raw + 1.3) * 100
        xb = -0.000691 * xa**2 - 0.000094 * ya**2 - 0.000593 * xa * ya + 1.227159 * xa + 0.125918 * ya - 18.666358
        yb = +0.000421 * xa**2 + 0.000106 * ya**2 - 0.000190 * xa * ya - 0.058721 * xa + 0.934087 * ya + 26.139245
        return xb, yb
    h = config_cam2["camera_height"]
    h_robot = config_cam2["robot_height"]
    val = (x_cam ** 2 + z_cam ** 2) - (h - h_robot) ** 2
    if val < 0:
        return None, None
    x_raw = math.sqrt(val)
    y_raw = y_cam
    xa = 360 - (x_raw - 1) * 100.0
    ya = (1.3 - y_raw) * 100.0
    xb = +0.000450 * xa**2 + 0.000391 * ya**2 + 0.000595 * xa * ya + 0.639393 * xa - 0.329119 * ya + 77.619781
    yb = +0.000108 * xa**2 - 0.000156 * ya**2 + 0.000433 * xa * ya - 0.064196 * xa + 0.862848 * ya + 2.782101
    return xb, yb


def _compute_new_camera_matrix(mtx, dist, w, h):
    new_mtx, _ = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))
    return new_mtx


def _compute_k_rotated(new_mtx, rotation, orig_h, orig_w):
    k_mat = np.array(new_mtx, dtype=np.float64)
    if rotation == "cw":
        t_mat = np.array([[0, -1, orig_h - 1], [1, 0, 0], [0, 0, 1]], dtype=np.float64)
    elif rotation == "ccw":
        t_mat = np.array([[0, 1, 0], [-1, 0, orig_w - 1], [0, 0, 1]], dtype=np.float64)
    else:
        return k_mat.copy()
    return t_mat @ k_mat


def _compute_camera_extrinsics(corner_pixels_sorted, k_rot, field_corners_3d):
    img_pts = np.array(corner_pixels_sorted, dtype=np.float64).reshape(-1, 1, 2)
    k_mat = np.array(k_rot, dtype=np.float64)
    success, rvec, tvec = cv2.solvePnP(
        np.asarray(field_corners_3d, dtype=np.float64),
        img_pts,
        k_mat,
        np.zeros(5, dtype=np.float64),
        flags=cv2.SOLVEPNP_ITERATIVE,
    )
    if not success:
        return None
    r_mat, _ = cv2.Rodrigues(rvec)
    cam_pos = -r_mat.T @ tvec
    return r_mat, cam_pos


def _ray_cast_to_height(px, py, k_rot, r_mat, cam_pos, height_cm):
    k_inv = np.linalg.inv(np.array(k_rot, dtype=np.float64))
    pt_h = np.array([px, py, 1.0], dtype=np.float64)
    ray_cam = k_inv @ pt_h
    ray_world = r_mat.T @ ray_cam
    cam_z = cam_pos[2, 0]
    ray_z = ray_world[2]
    if abs(ray_z) < 1e-10:
        return None, None
    t_val = (height_cm - cam_z) / ray_z
    if t_val < 0:
        return None, None
    point = cam_pos.flatten() + t_val * ray_world
    return float(point[0]), float(point[1])


def _is_finite_xy(x, y):
    return x is not None and y is not None and math.isfinite(float(x)) and math.isfinite(float(y))


def _should_accept_ray_xy(ray_x, ray_y, mapped_x, mapped_y, field_width, field_height, margin_cm, max_deviation_cm):
    if not _is_finite_xy(ray_x, ray_y):
        return False

    rx = float(ray_x)
    ry = float(ray_y)
    if rx < -float(margin_cm) or rx > (float(field_width) + float(margin_cm)):
        return False
    if ry < -float(margin_cm) or ry > (float(field_height) + float(margin_cm)):
        return False

    if _is_finite_xy(mapped_x, mapped_y):
        dist = math.hypot(rx - float(mapped_x), ry - float(mapped_y))
        if dist > float(max_deviation_cm):
            return False
    return True


def _rotate_pixel_to_rotated_frame(x, y, cam_role, frame_h=480, frame_w=640):
    if cam_role == "Cam1":
        return float(frame_h - 1 - y), float(x)
    return float(y), float(frame_w - 1 - x)


def _compute_angle_from_R(r_mat):
    forward = r_mat[:, 1]
    ang = math.degrees(math.atan2(forward[0], forward[1]))
    return _normalize_angle(ang)
