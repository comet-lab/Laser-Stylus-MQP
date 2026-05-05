import numpy as np
from typing import Dict, Any

def polygon_to_grid_map(
    pts_xy: np.ndarray,
    square_size: float,
    padding: float = 0.0,
    include_boundary: bool = True,
) -> Dict[str, Any]:
    """
    Given 4 points in meters (x,y), generate a grid validity map where True
    means the cell center lies inside the polygon.

    Parameters
    ----------
    pts_xy : (4,2) array-like
        Polygon vertices in world meters (x,y). Can be convex/concave but must be simple.
    square_size : float
        Grid cell size in meters.
    padding : float
        Extra margin (meters) added around polygon bbox before discretization.
    include_boundary : bool
        If True, points on boundary are treated as inside.

    Returns
    -------
    out : dict with keys
        "map"       -> np.ndarray bool shape (H,W)
        "size"      -> float (square_size)
        "origin" -> (ox, oy) world coord (meters) of cell (0,0) *corner* (lower-left)
    """
    pts_xy = np.asarray(pts_xy, dtype=float)
    if pts_xy.shape != (4, 2):
        raise ValueError(f"pts_xy must have shape (4,2), got {pts_xy.shape}")
    if square_size <= 0:
        raise ValueError("square_size must be > 0")

    from matplotlib.path import Path

    pts = np.asarray(pts_xy, dtype=float)
    pts_closed = np.vstack([pts, pts[0]])     # 5th point is dummy for CLOSEPOLY
    poly_path = Path(pts_closed, closed=True)

    minx, miny = pts_xy.min(axis=0) - padding
    maxx, maxy = pts_xy.max(axis=0) + padding

    # Define origin as the LOWER-LEFT corner of the grid (cell (0,0) corner)
    ox = float(minx)
    oy = float(miny)

    W = int(np.ceil((maxx - ox) / square_size))
    H = int(np.ceil((maxy - oy) / square_size))
    W = max(W, 1)
    H = max(H, 1)

    # Cell centers in world coordinates:
    xs = ox + (np.arange(W) + 0.5) * square_size
    ys = oy + (np.arange(H) + 0.5) * square_size

    # Meshgrid of centers
    XX, YY = np.meshgrid(xs, ys)  # shapes (H,W)
    centers = np.column_stack([XX.ravel(), YY.ravel()])  # (H*W,2)

    radius = 1e-12 if include_boundary else 0.0
    inside = poly_path.contains_points(centers, radius=radius).reshape(H, W)

    return {
        "map": inside,
        "size": float(square_size),
        "origin": (ox, oy),
    }