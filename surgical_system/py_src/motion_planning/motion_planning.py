from PIL import Image
import numpy as np
import cv2
import matplotlib.pyplot as plt
from shapely.geometry import Polygon, LineString, MultiLineString, GeometryCollection
from shapely.ops import unary_union
from shapely.validation import make_valid
from shapely.affinity import rotate, translate


class Motion_Planner():
    @staticmethod
    def _row_intervals(row: np.ndarray):
        r = (row > 0).astype(np.uint8)
        trans = np.diff(np.concatenate(([0], r, [0])))
        starts = np.where(trans == 1)[0]
        ends   = np.where(trans == -1)[0]
        return list(zip(starts, ends))  # [ [a,b), ... ]

    def raster_pattern(mask: np.ndarray, pitch: int = 6):
        """
        Continuous boustrophedon path inside a filled mask (no holes).
        mask: HxW, 1/True=inside, 0/False=outside
        pitch: scanline spacing in pixels
        Returns: list[(x,y)] polyline in pixel units.
        """
        H, W = mask.shape
        ys = list(range(0, H, pitch))
        spans_by_row = [Motion_Planner._row_intervals(mask[y]) for y in ys]

        path = []
        prev_idx = None      # index of last non-empty stripe
        stripe_count = 0     # number of non-empty stripes emitted (for serpentine parity)

        for i, (y, spans) in enumerate(zip(ys, spans_by_row)):
            if not spans:    # empty stripe
                continue

            spans.sort(key=lambda ab: ab[1]-ab[0], reverse=True)
            a, b = spans[0]

            if prev_idx is None:
                # First stripe: go L->R
                path.append((a, y))
                path.append((b, y))
                prev_idx = i
                stripe_count = 1
                continue


            ap, bp = sorted(spans_by_row[prev_idx], key=lambda ab: ab[1]-ab[0], reverse=True)[0]
            yp = ys[prev_idx]

            if (stripe_count - 1) % 2 == 0:
                # Last stripe ended at bp (L->R). This stripe goes R->L.
                x_conn = min(bp, b)         # stay inside overlap on the right
                if path[-1] != (x_conn, yp):
                    path.append((x_conn, yp))
                path.append((x_conn, y))
                if x_conn != b:
                    path.append((b, y))
                path.append((a, y))         # traverse R->L
            else:
                # Last stripe ended at ap (R->L). This stripe goes L->R.
                x_conn = max(ap, a)         # stay inside overlap on the left
                if path[-1] != (x_conn, yp):
                    path.append((x_conn, yp))
                path.append((x_conn, y))
                if x_conn != a:
                    path.append((a, y))
                path.append((b, y))         # traverse L->R

            prev_idx = i
            stripe_count += 1

        # Deduplicate exact duplicates
        out = []
        for p in path:
            if not out or out[-1] != p:
                out.append(p)
        return np.array(out)
    
    
    @staticmethod
    def _create_polygon(img: np.ndarray, epsilon = 0.1):
        
        contours, _ = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        
        edge = max(contours, key=cv2.contourArea)
        perimeter_pts = edge[:, 0, :] 
        plt.plot(perimeter_pts[:, 0], perimeter_pts[:, 1])
        plt.show()
        
        poly = Polygon(perimeter_pts)

        if not poly.is_valid:
            try:
                poly = make_valid(poly)
            except Exception:
                poly = poly.buffer(0)
                
        if poly.geom_type == "MultiPolygon":
            poly = max(poly.geoms, key=lambda g: g.area)

        # Simplify (keeps general shape, reduces noise)
        # poly = poly.simplify(simplify_eps, preserve_topology=True)

        return poly, perimeter_pts    
    
    @staticmethod
    def _extract_lines(geom):
        """Return a list of LineString segments from shapely intersection output."""
        if geom.is_empty:
            return []
        if isinstance(geom, LineString):
            return [geom]
        if isinstance(geom, MultiLineString):
            return list(geom.geoms)
        if isinstance(geom, GeometryCollection):
            out = []
            for g in geom.geoms:
                out.extend(Motion_Planner._extract_lines(g))
            return out
        return []
    
    @staticmethod
    def poly_raster(poly, spacing: float, theta_deg: float = 0.0, margin: float = 0.0):
        """
        Returns ordered LineString segments in boustrophedon (zig-zag) order.
        spacing: distance between adjacent raster lines (pixels).
        theta_deg: raster angle in degrees.
        margin: optional inward offset (pixels), e.g., half line width.
        """
        # Optional inward offset so lines don't spill over boundary
        work = poly.buffer(-margin) if margin > 0 else poly
        if work.is_empty:
            return []

        # Rotate polygon so scanlines are horizontal
        work_r = rotate(work, -theta_deg, origin="centroid", use_radians=False)
        minx, miny, maxx, maxy = work_r.bounds

        # Big scanline length across bounds
        L = (maxx - minx) + 2 * spacing

        segments = []
        y = miny - (miny % spacing)  # snap to grid-ish

        flip = False
        while y <= maxy + 1e-9:
            scan = LineString([(minx - spacing, y), (maxx + spacing, y)])
            inter = work_r.intersection(scan)
            lines = Motion_Planner._extract_lines(inter)

            # Clean tiny segments
            lines = [ln for ln in lines if ln.length > 1e-6]

            # Sort left->right by x of start
            lines.sort(key=lambda ln: ln.coords[0][0])

            # Zig-zag ordering: reverse every other row
            if flip:
                lines = [LineString(list(ln.coords)[::-1]) for ln in reversed(lines)]

            segments.extend(lines)
            flip = not flip
            y += spacing

        # Rotate back
        segments = [rotate(ln, theta_deg, origin=work.centroid, use_radians=False) for ln in segments]
        return Motion_Planner.segments_to_waypoints(segments, jump_threshold=5.0)
    
    @staticmethod
    def segments_to_waypoints(segments, jump_threshold=2.0):
        """
        Convert list of LineStrings into Nx2 numpy array of waypoints.
        jump_threshold: if distance between end and next start is large,
                        insert start point explicitly.
        Returns:
            waypoints: (N,2) float array
        """
        pts_out = []
        last_end = None

        for seg in segments:
            coords = np.asarray(seg.coords, dtype=float)  # (M,2)

            start = coords[0]
            end   = coords[-1]

            if last_end is None:
                pts_out.append(coords)
            else:
                gap = np.linalg.norm(start - last_end)

                if gap > jump_threshold:
                    pts_out.append(start[None, :])

                pts_out.append(coords)

            last_end = end

        if len(pts_out) == 0:
            return np.zeros((0, 2))

        return np.vstack(pts_out)

    
    
    
    @staticmethod
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
    

def main():
    img_path = "surgical_system/py_src/motion_planning/path1.png"
    img = np.array(Image.open(img_path))
    img = Motion_Planner.fill_in_shape(img)
    img = cv2.resize(img, (1280, 720), interpolation=cv2.INTER_NEAREST)
    if img.ndim == 3 and img.shape[2] == 4:
        gray = cv2.cvtColor(img, cv2.COLOR_BGRA2GRAY)
    elif img.ndim == 3:
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    else:
        gray = img
        
    polygon, edge = Motion_Planner._create_polygon(gray)
    # plot_polygon(polygon, show_vertices=True)
    waypoints = Motion_Planner.poly_raster(
        polygon,
        spacing=35.0,        # pixels
        theta_deg=45.0,      # angle
        margin=10.0          # inward offset
    )

    # waypoints = Motion_Planner.segments_to_waypoints(segments, jump_threshold=5.0)
    plt.plot(waypoints[:, 0], waypoints[:, 1])
    plt.plot(edge[:, 0], edge[:, 1])
    plt.show()
    
    
    # path = Motion_Planner.raster_pattern(img, pitch = 10)
    # # 4) Visualize the path overlayed on the mask
    # print("img size: ", np.shape(img))
    # fig, ax = plt.subplots(figsize=(8,4))
    # # ax.imshow(img, cmap='gray')
    # if len(path) > 1:
    #     xs = [p[0] for p in path]
    #     ys_plot = [p[1] for p in path]
    #     ax.plot(xs, ys_plot, linewidth=1)  # default color
    # ax.set_axis_off()
    # plt.show()
    
    # print(path)

if __name__=='__main__':
    main()
    