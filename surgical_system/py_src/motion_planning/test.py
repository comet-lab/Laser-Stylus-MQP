import cv2
import numpy as np
import matplotlib.pyplot as plt
import pyclipper

from shapely.geometry import Polygon, LineString, MultiLineString, Point, MultiPolygon, LinearRing
from shapely.ops import unary_union, nearest_points
from shapely.prepared import prep
from matplotlib.collections import LineCollection


# -----------------------------
# Input: image -> Polygon (no holes)
# -----------------------------

def fill_in_shape(img):
        if img.ndim == 3 and img.shape[2] == 4:
            gray = cv2.cvtColor(img, cv2.COLOR_BGRA2GRAY)
        elif img.ndim == 3:
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        else:
            gray = img

        # Binary edge map
        _, edges = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)

        # Find the largest contour and fill it to get a solid mask
        cnts, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        mask = np.zeros_like(gray, dtype=np.uint8)
        if cnts:
            # assume largest contour is the oval
            c = max(cnts, key=cv2.contourArea)
            cv2.drawContours(mask, [c], -1, 255, thickness=cv2.FILLED)

        kernel = np.ones((3,3), np.uint8)
        mask_in = cv2.erode(mask, kernel, iterations=1)
        return mask_in 
    
def polygon_from_binary_image(path, thresh=127, simplify_eps=1.0):
    img = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
    img = fill_in_shape(img)
    if img is None:
        raise FileNotFoundError(path)

    _, bw = cv2.threshold(img, thresh, 255, cv2.THRESH_BINARY)
    contours, _ = cv2.findContours(bw, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    if not contours:
        raise ValueError("No contour found in image")

    c = max(contours, key=cv2.contourArea).reshape(-1, 2).astype(float)
    poly = Polygon(c)
    if not poly.is_valid:
        poly = poly.buffer(0)

    if simplify_eps and simplify_eps > 0:
        poly = poly.simplify(simplify_eps, preserve_topology=True)

    if poly.geom_type != "Polygon":
        # if MultiPolygon, keep largest
        poly = max(list(poly.geoms), key=lambda p: p.area)

    return poly


# -----------------------------
# Clipper offset helpers (robust contour offsets)
# -----------------------------
def _to_clipper_path(coords, scale):
    return [(int(round(x * scale)), int(round(y * scale))) for x, y in coords]

def _from_clipper_path(path, scale):
    return [(x / scale, y / scale) for x, y in path]

def clipper_offset(paths, delta, join_type=pyclipper.JT_SQUARE):
    co = pyclipper.PyclipperOffset()
    co.AddPaths(paths, join_type, pyclipper.ET_CLOSEDPOLYGON)
    return co.Execute(delta)

def clipper_paths_to_polys(paths, scale, area_tol = 5):
    polys = []
    for p in paths:
        pts = _from_clipper_path(p, scale)
        if len(pts) >= 3:
            P = Polygon(pts)
            if not P.is_valid:
                P = P.buffer(0)
            if P.is_valid and P.area > area_tol:
                # print(P.area)
                polys.append(P)
    return polys


# -----------------------------
# Primary fill: inward contour-parallel loops
# -----------------------------
def inward_offset_loops(poly: Polygon, d: float, scale=2000, join_type=pyclipper.JT_SQUARE, margin=100.0):
    """
    Generates nested inward offset polygons spaced roughly by d.
    Returns list of layers: [ [Polygon, Polygon, ...], ... ] (outer -> inner)
    """
    work = poly.buffer(-margin) if margin > 0 else poly
    if work.is_empty:
        return []

    # Only outer boundary; no holes assumed
    outer = [ _to_clipper_path(list(work.exterior.coords), scale) ]
    delta = int(round(d * scale))

    layers = []
    current = outer
    while True:
        polys = clipper_paths_to_polys(current, scale, area_tol=5)
        if not polys:
            break
        layers.append(polys)

        nxt = clipper_offset(current, -delta, join_type=join_type)
        if not nxt:
            break
        current = nxt
        plot_polygon(to_multipolygon(polys))
    
    return layers

def to_multipolygon(polys):
    """
    polys: iterable of shapely Polygon (or things convertible to Polygon)
    returns: shapely MultiPolygon
    """
    cleaned = []
    for p in polys:
        if p is None:
            continue
        if p.geom_type == "Polygon":
            q = p
        elif p.geom_type == "MultiPolygon":
            cleaned.extend(list(p.geoms))
            continue
        else:
            raise TypeError(f"Expected Polygon/MultiPolygon, got {p.geom_type}")

        if not q.is_valid:
            q = q.buffer(0)
        if (not q.is_empty) and q.area > 0:
            cleaned.append(q)

    return MultiPolygon(cleaned)

def plot_polygon(poly, ax=None, color='blue', alpha=0.4, show_vertices=False):
    if ax is None:
        fig, ax = plt.subplots()

    if poly.geom_type == "Polygon":
        polys = [poly]
    elif poly.geom_type == "MultiPolygon":
        polys = list(poly.geoms)
    else:
        raise ValueError("Not a polygon")

    for p in polys:
        # Exterior
        x, y = p.exterior.xy
        ax.fill(x, y, color=color, alpha=alpha)
        ax.plot(x, y, color='black')

        # Holes
        for interior in p.interiors:
            xi, yi = interior.xy
            ax.fill(xi, yi, color='white')
            ax.plot(xi, yi, color='black')

        if show_vertices:
            ax.scatter(x, y, s=10, color='red')

    ax.set_aspect('equal')
    ax.set_title("Shapely Polygon")
    plt.show()
    

# -----------------------------
# Connectivity: stitch many closed loops into one continuous path
# -----------------------------

def _orientation(p, q, r):
    return np.sign((q[0]-p[0])*(r[1]-p[1]) - (q[1]-p[1])*(r[0]-p[0]))

def _on_segment(p, q, r):
    return (min(p[0], r[0]) <= q[0] <= max(p[0], r[0]) and
            min(p[1], r[1]) <= q[1] <= max(p[1], r[1]))

def _segments_intersect(p1, p2, q1, q2):
    o1 = _orientation(p1, p2, q1)
    o2 = _orientation(p1, p2, q2)
    o3 = _orientation(q1, q2, p1)
    o4 = _orientation(q1, q2, p2)

    if o1 != o2 and o3 != o4:
        return True

    # collinear overlap
    if o1 == 0 and _on_segment(p1, q1, p2): return True
    if o2 == 0 and _on_segment(p1, q2, p2): return True
    if o3 == 0 and _on_segment(q1, p1, q2): return True
    if o4 == 0 and _on_segment(q1, p2, q2): return True

    return False

def connector_crosses_path(a, b, path_pts):
    """
    a, b: (2,) numpy arrays
    path_pts: (N,2) numpy array
    Returns True if segment a->b intersects any
    existing segment in path (excluding last segment).
    """

    if len(path_pts) < 2:
        return False

    # Check against all segments except the last one
    # (ignore adjacency)
    for i in range(len(path_pts) - 2):
        q1 = path_pts[i]
        q2 = path_pts[i + 1]

        # Ignore shared endpoint cases
        if np.allclose(a, q2) or np.allclose(b, q1):
            continue

        if _segments_intersect(a, b, q1, q2):
            return True

    return False

def _straight_connector_ok(poly_prepared, a, b):
    return poly_prepared.covers(LineString([a, b]))

def _boundary_walk_connector(ext: LinearRing, a, b, step=2.0):
    """
    Connector that stays on/inside polygon:
    a -> nearest boundary pa -> walk along boundary -> pb -> b.
    Choose shorter direction around boundary.
    """
    da = ext.project(Point(a))
    db = ext.project(Point(b))
    L = ext.length

    pa = ext.interpolate(da)
    pb = ext.interpolate(db)

    def sample_along(d1, d2):
        # forward with wrap
        if d2 >= d1:
            ds = np.linspace(d1, d2, max(2, int((d2 - d1) / step) + 2))
            pts = [ext.interpolate(t).coords[0] for t in ds]
        else:
            ds1 = np.linspace(d1, L, max(2, int((L - d1) / step) + 2))
            ds2 = np.linspace(0, d2, max(2, int(d2 / step) + 2))
            pts = [ext.interpolate(t).coords[0] for t in ds1] + [ext.interpolate(t).coords[0] for t in ds2]
        return pts

    fwd = sample_along(da, db)
    bwd = list(reversed(sample_along(db, da)))  # pa->pb the other way

    walk = fwd if LineString(fwd).length <= LineString(bwd).length else bwd
    return [tuple(a), tuple(pa.coords[0])] + walk[1:-1] + [tuple(pb.coords[0]), tuple(b)]

def _open_ring_coords(ring_coords):
    coords = list(ring_coords)
    if np.allclose(coords[0], coords[-1]):
        coords = coords[:-1]
    return coords

def _rotate_to_start(coords, start_pt):
    pts = np.asarray(coords, float)
    k = int(np.argmin(np.sum((pts - np.asarray(start_pt))**2, axis=1)))
    return coords[k:] + coords[:k]

def _take_prefix_by_length(coords, L):
    """
    Take a polyline prefix from coords (closed ring opened) up to arc-length L.
    Returns list of points (includes the last interpolated point).
    """
    if len(coords) < 2:
        return coords
    out = [coords[0]]
    acc = 0.0
    for i in range(1, len(coords)+1):  # +1 to allow wrap segment
        a = np.asarray(coords[i-1], float)
        b = np.asarray(coords[i % len(coords)], float)
        seg = np.linalg.norm(b - a)
        if acc + seg >= L:
            t = (L - acc) / (seg + 1e-12)
            p = a + t * (b - a)
            out.append(tuple(p))
            return out
        else:
            out.append(tuple(b))
            acc += seg
    return out

def nearest_layers_order(layers, start_pt):
    s = Point(start_pt)

    remaining = [p for layer in layers for p in layer]
    all_loops = []

    current = s
    while remaining:
        # pick polygon whose exterior is closest to current point
        best_i, best_d = None, float("inf")
        for i, p in enumerate(remaining):
            d = p.exterior.distance(current)
            if d < best_d:
                best_d, best_i = d, i

        p = remaining.pop(best_i)
        all_loops.append(p.exterior)

        # update current point to the closest point on this loop (good proxy)
        proj = p.exterior.project(current)
        cp = p.exterior.interpolate(proj)
        current = Point(cp.x, cp.y)

    return all_loops


def stitch_loops(poly, layers, d, connector_step=10.0, stop_margin=None):
    """
    Material-aware stitching:
      - DOES NOT close each loop
      - traverses most of each loop, but stops ~stop_margin before returning to seam
      - transitions to next loop at nearest point
    """
    if stop_margin is None:
        stop_margin = d  # default: stop one "bead width" before closing

    poly_p = prep(poly)

    # flatten loops (same as before)
    all_loops = nearest_layers_order(layers, Point(layers[0][0].exterior.coords[0]))
    print(len(layers))
    # all_loops = []
    # for layer in layers:
    #     layer_sorted = sorted(layer, key=lambda p: p.area, reverse=True)
    #     for p in layer_sorted:
    #         all_loops.append(p.exterior)

    if not all_loops:
        return []


    path = []
    last = None

    for idx, ring in enumerate(all_loops):
        ring_coords = _open_ring_coords(ring.coords)

        # pick start point on this ring
        if last is None:
            start = ring_coords[0]
        else:
            ls = LineString(list(ring.coords))
            start = nearest_points(Point(last), ls)[1].coords[0]

        # rotate ring so it starts at start
        open_loop = _rotate_to_start(ring_coords, start)

        # connector from last -> start
        if last is not None and (not np.allclose(last, open_loop[0])):
            np_path = np.array(path)
            plt.plot(np_path[:, 0], np_path[:, 1])
            plt.scatter(last[0], last[1])
            plt.scatter(open_loop[0][0], open_loop[0][1])
            plt.show()
            if (_straight_connector_ok(poly_p, last, open_loop[0]) and not connector_crosses_path(last, open_loop[0], np.array(path))):
                path.append(open_loop[0])
            else:
                conn = _boundary_walk_connector(poly.exterior, last, open_loop[0], step=connector_step)
                if np.allclose(path[-1], conn[0]):
                    path.extend(conn[1:])
                else:
                    path.extend(conn)
            np_path = np.array(path)
            # plt.plot(np_pq
            

        # traverse this ring, but stop early by stop_margin
        ring_ls = LineString(open_loop + [open_loop[0]])  # closed for length measurement
        L = ring_ls.length
        traverse_L = max(0.0, L - stop_margin)

        arc = _take_prefix_by_length(open_loop, traverse_L)
        # append arc (avoid dup)
        if not path or not np.allclose(path[-1], arc[0]):
            path.append(arc[0])
        path.extend(arc[1:])
        
        ## Showing path
        # np_path = np.array(path)
        # plt.plot(np_path[:, 0], np_path[:, 1])
        # plt.show()

        last = path[-1]

    return path


# -----------------------------
# Residual gaps: compute uncovered pockets and fill with local zigzag
# -----------------------------
def toolpath_coverage_region(path_pts, radius):
    """
    Approximate swept area: buffer the polyline by radius.
    """
    if len(path_pts) < 2:
        return Polygon()
    return LineString(path_pts).buffer(radius, cap_style=2, join_style=2)

def pca_angle(coords):
    """Return dominant axis angle (radians) from PCA on point cloud."""
    X = np.array(coords)
    X = X - X.mean(axis=0, keepdims=True)
    C = X.T @ X
    w, v = np.linalg.eigh(C)
    d = v[:, np.argmax(w)]
    return float(np.arctan2(d[1], d[0]))

def zigzag_fill_region(region: Polygon, d: float, angle_rad: float):
    """
    Create boustrophedon zigzag lines inside region at spacing d, oriented by angle_rad.
    Returns one continuous polyline covering all segments in that region (may need connectors).
    """
    # Rotate region so scanlines horizontal
    from shapely.affinity import rotate
    region_r = rotate(region, -np.degrees(angle_rad), origin=region.centroid, use_radians=False)
    minx, miny, maxx, maxy = region_r.bounds
    ys = np.arange(miny, maxy + d, d)

    segs = []
    pad = (maxx - minx) * 0.05 + d
    for y in ys:
        scan = LineString([(minx - pad, y), (maxx + pad, y)])
        inter = region_r.intersection(scan)
        if inter.is_empty:
            continue
        if isinstance(inter, LineString):
            if inter.length > 1e-9:
                segs.append(inter)
        elif isinstance(inter, MultiLineString):
            segs.extend([g for g in inter.geoms if g.length > 1e-9])

    if not segs:
        return []

    # Sort by y then x
    segs.sort(key=lambda s: (s.centroid.y, s.centroid.x))

    # Stitch segments boustrophedon with straight connectors (region is small)
    path = []
    last = None
    flip = False
    region_p = prep(region_r)

    def endpoints(s):
        c = list(s.coords)
        return c[0], c[-1]

    for s in segs:
        a, b = endpoints(s)
        start, end = (a, b) if not flip else (b, a)

        if last is None:
            path.append(start)
        else:
            if not np.allclose(last, start):
                # connect inside rotated region if possible; else just jump (rare for single pocket)
                if region_p.covers(LineString([last, start])):
                    path.append(start)
                else:
                    path.append(start)
        path.append(end)
        last = end
        flip = not flip

    # Rotate path back
    ls = LineString(path)
    ls = rotate(ls, np.degrees(angle_rad), origin=region.centroid, use_radians=False)
    return list(ls.coords)


def fill_residual_with_zigzag(poly: Polygon, base_path, d: float, rounds=2):
    """
    Iteratively fill remaining uncovered areas with local zigzag, stitched onto the base path.
    """
    poly_p = prep(poly)
    path = list(base_path)

    for _ in range(rounds):
        cover = toolpath_coverage_region(path, radius=d / 2.0)
        residual = poly.difference(cover)
        if residual.is_empty:
            break

        # residual can be MultiPolygon
        pockets = [residual] if residual.geom_type == "Polygon" else list(residual.geoms)
        pockets = [p for p in pockets if p.area > (d * d) * 0.5]  # ignore tiny specks
        if not pockets:
            break

        # Fill each pocket with local zigzag and stitch to current path
        for pocket in pockets:
            angle = pca_angle(list(pocket.exterior.coords))
            zz = zigzag_fill_region(pocket, d=d, angle_rad=angle)
            if len(zz) < 2:
                continue

            # Stitch: connect current path end -> zigzag start (inside poly)
            last = path[-1]
            start = zz[0]
            if not np.allclose(last, start):
                if poly_p.covers(LineString([last, start])):
                    path.append(start)
                else:
                    conn = _boundary_walk_connector(poly, last, start, step=max(1.0, d / 4.0))
                    if np.allclose(path[-1], conn[0]):
                        path.extend(conn[1:])
                    else:
                        path.extend(conn)
            path.extend(zz)

    return path


# -----------------------------
# End-to-end planner
# -----------------------------
def plan_trajectory_for_shape(poly: Polygon, d: float, margin=0.0, connector_step=2.0, zigzag_rounds=2):
    # 1) contour-parallel inward loops
    layers = inward_offset_loops(poly, d=d, margin=margin)

    # 2) stitch loops into one continuous path
    base = stitch_loops(poly, layers, d, connector_step=connector_step)

    # 3) fill remaining pockets with local zigzag + stitch
    # full = fill_residual_with_zigzag(poly, base, d=d, rounds=zigzag_rounds)

    return base


def plot_poly_and_path(poly: Polygon, path_pts, title="Planned Trajectory"):
    fig, ax = plt.subplots(figsize=(9, 7))

    # ---- Plot polygon ----
    x, y = poly.exterior.xy
    ax.plot(x, y, color='black', linewidth=2)

    # ---- Plot gradient path ----
    if path_pts is not None and len(path_pts) >= 2:
        pts = np.asarray(path_pts, dtype=float)

        # Create line segments between consecutive points
        segments = np.stack([pts[:-1], pts[1:]], axis=1)

        # Color value increases along trajectory
        t = np.linspace(0, 1, len(segments))

        lc = LineCollection(
            segments,
            cmap='viridis',     # try 'plasma', 'inferno', 'jet'
            norm=plt.Normalize(0, 1),
            linewidth=2
        )
        lc.set_array(t)

        ax.add_collection(lc)

        # Optional: show start and end points
        ax.scatter(pts[0,0], pts[0,1], color='green', s=60, label='Start', zorder=5)
        ax.scatter(pts[-1,0], pts[-1,1], color='red', s=60, label='End', zorder=5)

    ax.set_aspect("equal", "box")
    ax.invert_yaxis()
    ax.axis("off")
    ax.set_title(title)
    plt.colorbar(lc, ax=ax, label="Path Progress")
    plt.show()


# -----------------------------
# Example usage
# -----------------------------
if __name__ == "__main__":
    img_path = "surgical_system\py_src\motion_planning\path2.png"   # <-- your input
    d = 10.0                 # <-- spacing between passes

    poly = polygon_from_binary_image(img_path, simplify_eps=1.0)
    path = plan_trajectory_for_shape(poly, d=d, margin=5.0, connector_step=max(1.0, d/4), zigzag_rounds=2)
    plot_poly_and_path(poly, path, title=f"Trajectory (d={d})")
