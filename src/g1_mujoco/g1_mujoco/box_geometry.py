"""Pure-numpy box pose estimation from RGB-D data.

The functions here know nothing about ROS or MuJoCo: they take an image mask,
a depth image and a camera model and return the tabletop box pose.  This keeps
the perception math testable and reusable on the physical robot, where the
same code runs on a real RGB-D stream.
"""
import math

import numpy as np

# Known cardboard box of the tabletop scene, metres (full extents).
BOX_DIMS = (0.255, 0.370, 0.090)
MASK_MIN_PIXELS = 80
TOP_INLIER_BAND = 0.012
YAW_SEARCH_STEP_DEG = 1.0
EXTENT_PERCENTILE = 2.0


def red_box_mask(rgb):
    """Return the boolean mask of the red box in an RGB uint8 image."""
    red, green, blue = (rgb[:, :, index] for index in range(3))
    mask = (red > 170) & (green < 110) & (blue < 110) & ((red.astype(np.int16) - green) > 90)
    return mask


def backproject(mask, depth, fx, fy, cx, cy):
    """Turn masked pixels plus optical-Z depth into optical-frame points."""
    pixels = np.argwhere(mask)
    if len(pixels) == 0:
        return np.zeros((0, 3), dtype=np.float64), pixels
    valid = np.isfinite(depth[mask]) & (depth[mask] > 0.10) & (depth[mask] < 4.0)
    pixels = pixels[valid]
    if len(pixels) == 0:
        return np.zeros((0, 3), dtype=np.float64), pixels
    v, u = pixels[:, 0].astype(np.float64), pixels[:, 1].astype(np.float64)
    z = depth[pixels[:, 0], pixels[:, 1]].astype(np.float64)
    xn, yn = (u - cx) / fx, (v - cy) / fy
    points = np.column_stack((xn * z, yn * z, z))
    return points, pixels


def transform_points(points, rotation, translation):
    if len(points) == 0:
        return points
    return points @ rotation.T + translation


def _robust_extents(projection):
    low = np.percentile(projection, EXTENT_PERCENTILE)
    high = np.percentile(projection, 100.0 - EXTENT_PERCENTILE)
    return high - low, 0.5 * (high + low)


def fit_box_pose(points, dims=BOX_DIMS, yaw_hint=None):
    """Fit the known-size box to 3D surface points given in a z-up frame.

    The camera mostly sees the top face, so the yaw is found by a deterministic
    search that minimises the mismatch between the rotated point-cloud extents
    and the known box footprint; the centre height follows from the top plane.
    Returns None when the cloud is too small to trust.  The yaw has a 180
    degree ambiguity which callers must treat through the returned axes, not
    the yaw sign.
    """
    if len(points) < MASK_MIN_PIXELS:
        return None
    z_top = float(np.median(points[:, 2]))
    top = points[np.abs(points[:, 2] - z_top) <= TOP_INLIER_BAND]
    if len(top) < MASK_MIN_PIXELS // 2:
        return None
    footprint = top[:, :2]
    long_full, short_full, height = dims[1], dims[0], dims[2]
    covariance = np.cov(footprint.T)
    values, vectors = np.linalg.eigh(covariance)
    long_xy = vectors[:, int(np.argmax(values))]
    yaw = float(np.arctan2(-long_xy[0], long_xy[1]))
    if yaw_hint is not None:
        alternative = yaw + math.pi if yaw < 0.0 else yaw - math.pi
        distance = lambda angle: abs(np.arctan2(np.sin(angle - yaw_hint), np.cos(angle - yaw_hint)))
        if distance(alternative) < distance(yaw):
            yaw = alternative
    cos, sin = np.cos(yaw), np.sin(yaw)
    long_axis = np.array([-sin, cos, 0.0])
    short_axis = np.array([cos, sin, 0.0])
    extent_long, centre_long = _robust_extents(footprint @ long_axis[:2])
    extent_short, centre_short = _robust_extents(footprint @ short_axis[:2])
    # Recover the world-frame centre from the box-frame medians.
    centre = short_axis[:2] * centre_short + long_axis[:2] * centre_long
    centre = np.array([centre[0], centre[1], z_top - height * 0.5])
    return {
        "centre": centre,
        "yaw": float(yaw),
        "long_axis": long_axis,
        "short_axis": short_axis,
        "extent_long": float(extent_long),
        "extent_short": float(extent_short),
        "residual": float(abs(extent_long - long_full) + abs(extent_short - short_full)),
        "top_z": z_top,
        "inliers": int(len(top)),
    }
