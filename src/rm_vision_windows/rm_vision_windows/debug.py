"""ROS-free equivalents of rm_tracker/node.cpp's two rqt image topics (RGB)."""

import cv2
import numpy as np


def project_camera(point, K, D):
    point = np.asarray(point, dtype=np.float64)
    if point.shape != (3,) or not np.isfinite(point).all() or point[2] <= 0:
        return None
    pixel = cv2.projectPoints(point.reshape(1, 3), np.zeros(3), np.zeros(3),
                             np.asarray(K, dtype=np.float64), np.asarray(D, dtype=np.float64))[0]
    pixel = pixel.reshape(2)
    return tuple(np.rint(pixel).astype(int)) if np.isfinite(pixel).all() and np.max(np.abs(pixel)) < 1e7 else None


def tracking_image(rgb, result, text_size=1.0):
    output = rgb.copy()
    for armor in result["observations"]:
        corners = np.rint(armor["corners_px"]).astype(np.int32)
        cv2.polylines(output, [corners[[0, 1, 3, 2]]], True, (255, 220, 0), 2)
    for armor in result["tracker_debug"]["yaw_armors"]:
        point = project_camera(armor["position_camera_m"], result["input"]["K"], result["input"]["D"])
        if point:
            cv2.putText(output, f"yaw {armor['yaw_rad']:.1f} rad", point,
                        cv2.FONT_HERSHEY_SIMPLEX, .55*text_size, (255, 0, 255), max(1, round(2*text_size)))
    return output


def ballistic_image(rgb, result, text_size=1.0):
    output = rgb.copy()
    height, width = output.shape[:2]
    cv2.drawMarker(output, (width//2, height//2), (255, 0, 255), cv2.MARKER_CROSS, 40, 2)
    debug, source = result["tracker_debug"], result["input"]
    for key, color in (("target_camera_m", (255, 0, 0)), ("muzzle_target_camera_m", (0, 255, 0))):
        pixel = project_camera(debug[key], source["K"], source["D"]) if key in debug else None
        if pixel:
            if key == "target_camera_m":
                cv2.circle(output, pixel, 18, color, 2)
            else:
                cv2.drawMarker(output, pixel, color, cv2.MARKER_CROSS, 25, 2)
    lines = [result["state"], "SPIN" if debug["spin"] else "NORMAL",
             f"Armor w: {debug.get('ekf_yaw_velocity_rad_s', 0):.2f} rad/s",
             f"Dist: {debug.get('target_distance_m', 0):.2f} m",
             f"Diff: {debug.get('target_angle_diff_deg', 0):.1f} deg",
             f"Pitch: {debug['gimbal_control'][1]:+.2f} deg",
             f"Yaw: {debug['gimbal_control'][0]:+.2f} deg",
             f"P/Y Tol: {debug['pitch_tolerance_deg']:.2f} / {debug['yaw_tolerance_deg']:.2f} deg",
             f"Bullet Speed: {debug['bullet_speed_m_s']:.2f} m/s",
             "FIRE ENABLE" if result["can_fire"] else "HOLD FIRE"]
    for row, text in enumerate(lines):
        point = (18, round((28+26*row)*text_size))
        color = (0, 255, 0) if row == len(lines)-1 and result["can_fire"] else (255, 255, 255)
        cv2.putText(output, text, point, cv2.FONT_HERSHEY_SIMPLEX, .6*text_size, (0, 0, 0), max(3, round(2*text_size)+2))
        cv2.putText(output, text, point, cv2.FONT_HERSHEY_SIMPLEX, .6*text_size, color, max(1, round(2*text_size)))
    return output


def observer_image(rgb, result, optical_transform, K, D):
    """Place the same tracker prediction into an observer image; no enemy truth."""
    output = rgb.copy()
    debug = result["tracker_debug"]
    if "target_camera_m" in debug:
        source = np.asarray(result["input"]["optical_transform"])
        world = source[:3, :3] @ debug["target_camera_m"] + source[:3, 3]
        target = optical_transform[:3, :3].T @ (world-optical_transform[:3, 3])
        point = project_camera(target, K, D)
        if point:
            cv2.circle(output, point, 12, (255, 0, 0), 2)
            cv2.putText(output, "tracker prediction", (point[0]+14, point[1]),
                        cv2.FONT_HERSHEY_SIMPLEX, .55, (255, 0, 0), 2)
    return output
