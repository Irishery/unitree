"""Pure-numpy bottle pose estimation from RGB-D data.

The functions here know nothing about ROS or MuJoCo: they take an image mask,
a depth image and a camera model and return the tabletop bottle pose.  This
keeps the perception math testable and reusable on the physical robot, where
the same code runs on a real RGB-D stream.
"""
import numpy as np

# Known bottle of the tabletop scene, metres.
BOTTLE_DIAMETER = 0.062
BOTTLE_HEIGHT = 0.235
MASK_MIN_PIXELS = 80


def red_bottle_mask(rgb):
    """Return the boolean mask of the red bottle body in an RGB uint8 image.

    The table is brown (green channel around 70-80), while the shaded red
    bottle body has a very low green/blue channel.  A low red threshold keeps
    the dark side of the cylinder in the mask; the green/blue gates reject the
    table and the blue neck/cap.
    """
    red, green, blue = (rgb[:, :, index] for index in range(3))
    mask = ((red > 80) & (green < 60) & (blue < 50)
            & ((red.astype(np.int16) - green) > 40))
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


def _fit_circle_xy(points):
    """Least-squares circle fit for the visible vertical cylinder surface."""
    x = points[:, 0]
    y = points[:, 1]
    a = np.column_stack((2.0 * x, 2.0 * y, np.ones_like(x)))
    b = x * x + y * y
    solution, _, _, _ = np.linalg.lstsq(a, b, rcond=None)
    centre_x, centre_y, offset = solution
    radius_sq = offset + centre_x * centre_x + centre_y * centre_y
    if not np.isfinite(radius_sq) or radius_sq <= 0.0:
        return None
    return float(centre_x), float(centre_y), float(np.sqrt(radius_sq))


def fit_bottle_pose(points, diameter=BOTTLE_DIAMETER, height=BOTTLE_HEIGHT):
    """Fit an upright cylinder to 3D surface points in a z-up frame.

    The bottle is rotationally symmetric, so the yaw is not observable and is
    returned as zero.  The centre of the visible body is recovered with a
    circle fit; the height follows from robust z extents.  Returns None when
    the cloud is too small to trust.
    """
    if len(points) < MASK_MIN_PIXELS:
        return None
    points = np.asarray(points, dtype=np.float64)
    finite = np.all(np.isfinite(points), axis=1)
    points = points[finite]
    if len(points) < MASK_MIN_PIXELS:
        return None
    z_low = float(np.percentile(points[:, 2], 1.0))
    z_high = float(np.percentile(points[:, 2], 98.0))
    height_fit = z_high - z_low
    if height_fit < max(0.02, 0.20 * height):
        return None
    span = height_fit
    # Use the lower main cylinder, below the white label and the shoulder
    # taper.  Fitting the shoulder would underestimate the radius and bias the
    # centre upward.
    mid_low = z_low + span * 0.05
    mid_high = z_low + span * 0.25
    mid = points[(points[:, 2] >= mid_low) & (points[:, 2] <= mid_high)]
    if len(mid) < MASK_MIN_PIXELS // 2:
        mid = points
    circle = _fit_circle_xy(mid)
    if circle is None:
        return None
    centre_x, centre_y, radius = circle
    # Accept any physically plausible bottle radius; the nominal parameter is
    # only a fallback for an ill-conditioned circle fit.
    fit_ok = 0.015 <= radius <= 0.080
    if not fit_ok:
        centre_x = float(np.median(mid[:, 0]))
        centre_y = float(np.median(mid[:, 1]))
        radius = 0.5 * float(diameter)
    # The visible bottom of the body is a stable reference; the centre follows
    # the measured height, so a different bottle does not need a new launch
    # parameter just to place the grasp height.
    centre_z = z_low + 0.5 * height_fit
    return {
        "centre": np.array([centre_x, centre_y, centre_z]),
        "yaw": 0.0,
        "diameter": float(2.0 * radius),
        "height": float(height_fit),
        "top_z": float(z_high),
        "inliers": int(len(mid)),
        "confidence": (min(1.0, len(mid) / 150.0) if fit_ok else 0.25),
        "residual": float(abs(2.0 * radius - diameter)),
    }
