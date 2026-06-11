"""Gate cross-section geometry: beam transforms, masks and extents.

Frame: x [m] across the belt (0 = center), z [m] above the belt (0 = belt
surface).  A laser pose places the scanner in this plane; the beam at
device angle ``a`` leaves at ``theta = rotation_deg + a`` (negated first
when the device is mounted mirrored).
"""

from __future__ import annotations

import math
from typing import List, Optional, Sequence, Tuple

from config import Pose, Roi, Sector


def beam_theta_deg(pose: Pose, angle_deg: float) -> float:
    device_angle = -angle_deg if pose.mirror else angle_deg
    return pose.rotation_deg + device_angle


def beam_point(pose: Pose, angle_deg: float, distance_m: float) -> Tuple[float, float]:
    theta = math.radians(beam_theta_deg(pose, angle_deg))
    return (pose.x_m + distance_m * math.cos(theta),
            pose.z_m + distance_m * math.sin(theta))


class TransformTable:
    """Precomputed per-beam unit vectors + sector mask for one scan layout.

    Build once per (pose, sector, beam angles) combination; projecting a
    scan is then two multiply-adds per beam.
    """

    def __init__(self, pose: Pose, sector: Sector,
                 angles_deg: Sequence[float]) -> None:
        self.pose = pose
        self.sector = sector
        self.angles_deg = list(angles_deg)
        self.mask: List[bool] = []
        self._ux: List[float] = []
        self._uz: List[float] = []
        for angle in self.angles_deg:
            self.mask.append(sector.min_deg <= angle <= sector.max_deg)
            theta = math.radians(beam_theta_deg(pose, angle))
            self._ux.append(math.cos(theta))
            self._uz.append(math.sin(theta))

    def project(self, distances_mm: Sequence[int],
                foreground: Optional[Sequence[bool]] = None,
                roi: Optional[Roi] = None) -> List[Tuple[float, float]]:
        """(x, z) points for valid, in-sector (optionally foreground/ROI) beams."""
        points: List[Tuple[float, float]] = []
        x0, z0 = self.pose.x_m, self.pose.z_m
        for i, distance in enumerate(distances_mm):
            if i >= len(self.mask):
                break
            if distance <= 0 or not self.mask[i]:
                continue
            if foreground is not None and not foreground[i]:
                continue
            d = distance / 1000.0
            x = x0 + d * self._ux[i]
            z = z0 + d * self._uz[i]
            if roi is not None and not roi.contains(x, z):
                continue
            points.append((x, z))
        return points


def foreground_mask(distances_mm: Sequence[int],
                    baseline_mm: Optional[Sequence[int]],
                    margin_mm: int) -> List[bool]:
    """True per beam where the echo is meaningfully closer than the baseline.

    No baseline: every valid echo counts.  A beam that had no baseline echo
    (0 mm = no return within range) but returns one now is foreground.
    """
    if baseline_mm is None:
        return [d > 0 for d in distances_mm]
    out: List[bool] = []
    for i, d in enumerate(distances_mm):
        if d <= 0:
            out.append(False)
            continue
        b = baseline_mm[i] if i < len(baseline_mm) else 0
        out.append(b <= 0 or d < b - margin_mm)
    return out


def trimmed_extent(values: Sequence[float],
                   trim: int) -> Optional[Tuple[float, float]]:
    """(low, high) after dropping ``trim`` outliers from each end."""
    if not values:
        return None
    ordered = sorted(values)
    if trim > 0 and len(ordered) > 2 * trim:
        ordered = ordered[trim:-trim]
    return ordered[0], ordered[-1]
