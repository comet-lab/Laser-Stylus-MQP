import numpy as np
import cv2
from typing import Dict, Tuple, Optional
import matplotlib.pyplot as plt
from scipy import ndimage
class DepthEstimation():
   
    @staticmethod
    def project_xy(H: np.ndarray, pts: np.ndarray, eps: float = 1e-12) -> np.ndarray:
        """
        Project 2D Euclidean points through a homography.

        Parameters
        ----------
        H : (3,3)
        pts : (N,2)

        Returns
        -------
        uv : (N,2)
        """
        pts = np.asarray(pts, dtype=float)
        ones = np.ones((pts.shape[0], 1), dtype=float)
        pts_h = np.hstack([pts, ones])                  # (N,3)

        p = (H @ pts_h.T).T                             # (N,3)
        denom = p[:, 2:3]                               # (N,1)

        uv = np.full((pts.shape[0], 2), np.inf, dtype=float)
        valid = np.abs(denom[:, 0]) >= eps
        uv[valid] = p[valid, :2] / denom[valid]
        return uv
    
    @staticmethod
    def estimate_depth_from_dense_stack(
        dense_stack: Dict[float, np.ndarray],
        obs_uvs: np.ndarray,
        cmd_XYs: np.ndarray,
        *,
        metric: str = "l2",
        refine: bool = True,
        min_valid_heights: int = 3,
        invert_homographies: bool = True,
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """
        Vectorized depth estimation for multiple observations/commands.

        Parameters
        ----------
        dense_stack : dict
            {z: H_z}, each H_z is (3,3)
        obs_uvs : (N,2)
            Observed pixel coordinates.
        cmd_XYs : (N,2)
            Commanded / known plane coordinates.
        metric : {'l2', 'l1'}
        refine : bool
            If True, do per-point local quadratic refinement after coarse batch search.
        min_valid_heights : int
        invert_homographies : bool
            If True, use inv(H_z) before projection, matching your original function.

        Returns
        -------
        z_hat : (N,)
        err_hat : (N,)
        uv_hat : (N,2)
        conf : (N,)
        """
        if len(dense_stack) < min_valid_heights:
            raise ValueError(f"dense_stack must have at least {min_valid_heights} entries.")

        obs_uvs = np.asarray(obs_uvs, dtype=float)
        cmd_XYs = np.asarray(cmd_XYs, dtype=float)

        if obs_uvs.ndim != 2 or obs_uvs.shape[1] != 2:
            raise ValueError("obs_uvs must have shape (N,2)")
        if cmd_XYs.ndim != 2 or cmd_XYs.shape[1] != 2:
            raise ValueError("cmd_XYs must have shape (N,2)")
        if obs_uvs.shape[0] != cmd_XYs.shape[0]:
            raise ValueError("obs_uvs and cmd_XYs must have the same number of rows")

        N = obs_uvs.shape[0]
        eps = 1e-12

        # ---- sort heights and stack homographies ----
        zs = np.array(sorted(dense_stack.keys()), dtype=float)          # (Z,)
        Hs = np.array([dense_stack[z] for z in zs], dtype=float)        # (Z,3,3)

        if invert_homographies:
            Hs = np.linalg.inv(Hs)                                      # (Z,3,3)

        Z = Hs.shape[0]

        X = cmd_XYs[:, 0]                                               # (N,)
        Y = cmd_XYs[:, 1]                                               # (N,)

        # ---- project all N points through all Z homographies ----
        # preds shape will be (N, Z, 2)
        u_num = (
            Hs[:, 0, 0][None, :] * X[:, None]
            + Hs[:, 0, 1][None, :] * Y[:, None]
            + Hs[:, 0, 2][None, :]
        )                                                               # (N,Z)

        v_num = (
            Hs[:, 1, 0][None, :] * X[:, None]
            + Hs[:, 1, 1][None, :] * Y[:, None]
            + Hs[:, 1, 2][None, :]
        )                                                               # (N,Z)

        denom = (
            Hs[:, 2, 0][None, :] * X[:, None]
            + Hs[:, 2, 1][None, :] * Y[:, None]
            + Hs[:, 2, 2][None, :]
        )                                                               # (N,Z)

        valid = np.abs(denom) >= eps

        u_pred = np.full((N, Z), np.inf, dtype=float)
        v_pred = np.full((N, Z), np.inf, dtype=float)

        u_pred[valid] = u_num[valid] / denom[valid]
        v_pred[valid] = v_num[valid] / denom[valid]

        # ---- residuals ----
        du = u_pred - obs_uvs[:, 0:1]                                   # (N,Z)
        dv = v_pred - obs_uvs[:, 1:2]                                   # (N,Z)

        if metric == "l2":
            errs = np.hypot(du, dv)                                     # (N,Z)
        elif metric == "l1":
            errs = np.abs(du) + np.abs(dv)
        else:
            raise ValueError("metric must be 'l2' or 'l1'")

        # ---- coarse discrete best height ----
        best_idx = np.argmin(errs, axis=1)                              # (N,)
        rows = np.arange(N)

        z_best = zs[best_idx].astype(float)                             # (N,)
        err_best = errs[rows, best_idx].astype(float)                   # (N,)
        uv_best = np.column_stack([
            u_pred[rows, best_idx],
            v_pred[rows, best_idx]
        ]).astype(float)                                                # (N,2)

        # ---- confidence proxy from best vs second-best ----
        if Z >= 2:
            part = np.partition(errs, 1, axis=1)
            err1 = part[:, 0]
            err2 = part[:, 1]
            conf = np.clip((err2 - err1) / np.maximum(err2, 1e-9), 0.0, 1.0)
        else:
            conf = np.zeros(N, dtype=float)

        if not refine or Z < 3:
            return z_best, err_best, uv_best, conf

        # ---- local refinement ----
        # Most of the speedup is already achieved. Refinement is done only on
        # each point's local winner neighborhood.
        z_out = z_best.copy()
        err_out = err_best.copy()
        uv_out = uv_best.copy()

        interior = (best_idx > 0) & (best_idx < Z - 1)
        refine_ids = np.where(interior)[0]

        for i in refine_ids:
            k = best_idx[i]

            z0 = float(zs[k - 1])
            z1 = float(zs[k])
            z2 = float(zs[k + 1])

            e0 = float(errs[i, k - 1])
            e1 = float(errs[i, k])
            e2 = float(errs[i, k + 1])

            d01 = z0 - z1
            d02 = z0 - z2
            d12 = z1 - z2

            a = (
                e0 / (d01 * d02)
                - e1 / (d01 * d12)
                + e2 / (d02 * d12)
            )

            b = (
                -e0 * (z1 + z2) / (d01 * d02)
                + e1 * (z0 + z2) / (d01 * d12)
                - e2 * (z0 + z1) / (d02 * d12)
            )

            if abs(a) < 1e-12:
                continue

            z_ref = -b / (2.0 * a)
            z_ref = float(np.clip(z_ref, min(z0, z2), max(z0, z2)))

            if z_ref <= z1:
                t = 0.0 if z1 == z0 else (z_ref - z0) / (z1 - z0)
                H_ref = (1.0 - t) * np.asarray(dense_stack[z0], float) + t * np.asarray(dense_stack[z1], float)
            else:
                t = 0.0 if z2 == z1 else (z_ref - z1) / (z2 - z1)
                H_ref = (1.0 - t) * np.asarray(dense_stack[z1], float) + t * np.asarray(dense_stack[z2], float)

            if invert_homographies:
                H_ref = np.linalg.inv(H_ref)

            uv_ref = DepthEstimation.project_xy(H_ref, cmd_XYs[i:i+1])
            u_ref, v_ref = uv_ref[0]

            du_i = u_ref - obs_uvs[i, 0]
            dv_i = v_ref - obs_uvs[i, 1]
            err_ref = float(np.hypot(du_i, dv_i)) if metric == "l2" else float(abs(du_i) + abs(dv_i))

            if np.isfinite(err_ref) and err_ref <= err_out[i]:
                z_out[i] = z_ref
                err_out[i] = err_ref
                uv_out[i] = [u_ref, v_ref]

        return z_out, err_out, uv_out, conf
    
    
    @staticmethod
    def load_homography_stack_npz(file_path):
        data = np.load(file_path)

        zs = data["zs"]       # (N,)
        Hs = data["Hs"]       # (N,3,3)

        return {float(z): H for z, H in zip(zs, Hs)}
    
    @staticmethod
    def normalize_homography(H, eps=1e-9):
        """
        Normalize homography to remove scale ambiguity
        """
        H = H.astype(float)
        
        if abs(H[2,2]) > eps:
            return H / H[2,2]
        else:
            norm = np.linalg.norm(H)
            if norm < eps:
                raise ValueError("Homography norm too small to normalize.")
            return H / norm

    @staticmethod
    def create_dense_stack(homography_stack: dict,
                        dz: float = 0.0001, # m
                        extrapolate: bool = False):
        """
        Create a dense homography stack via linear interpolation.
        
        Parameters
        ----------
        homography_stack : dict
            {height: 3x3 homography matrix}
        dz : float
            Height spacing for dense stack (same units as heights)
        extrapolate : bool
            If False, only interpolate within range
        
        Returns
        -------
        dense_stack : dict
            {height: interpolated normalized 3x3 homography}
        """
        
        # --- Step 1: Sort heights ---
        heights = np.array(sorted(homography_stack.keys()), dtype=float)
        
        Hs = [DepthEstimation.normalize_homography(homography_stack[z]) for z in heights]
        Hs = np.stack(Hs, axis=0)
        
        z_min, z_max = heights[0], heights[-1]
        
        # --- Step 2: Create dense grid ---
        dense_heights = np.arange(z_min, z_max + dz, dz)
        
        dense_stack = {}
        
        for z in dense_heights:
            
            # Exact match
            if z in homography_stack:
                dense_stack[z] = DepthEstimation.normalize_homography(homography_stack[z])
                continue
            
            # If outside bounds
            if z < z_min or z > z_max:
                if not extrapolate:
                    continue
                else:
                    # clamp to nearest
                    if z < z_min:
                        dense_stack[z] = Hs[0]
                    else:
                        dense_stack[z] = Hs[-1]
                    continue
            
            # --- Step 3: Find bracketing indices ---
            idx_upper = np.searchsorted(heights, z)
            idx_lower = idx_upper - 1
            
            z0 = heights[idx_lower]
            z1 = heights[idx_upper]
            
            H0 = Hs[idx_lower]
            H1 = Hs[idx_upper]
            
            # --- Step 4: Linear interpolation ---
            t = (z - z0) / (z1 - z0)
            H_interp = (1 - t) * H0 + t * H1
            
            # --- Step 5: Renormalize ---
            H_interp = DepthEstimation.normalize_homography(H_interp)
            
            dense_stack[z] = H_interp
        
        return dense_stack
    
    def generate_depth_mapping(positions: np.ndarray,
                               cell_size: float = 0.001,          # meters per cell (example: 1 mm)
                               # (xmin, xmax, ymin, ymax) in same units as x,y
                               bounds: Optional[Tuple[float, float, float, float]] = None,    
                               agg: str = "median",               # "median", "mean", "min", "max", "last"
                               return_meta: bool = True,
                               dtype=np.float32,):
        """
        Rasterize scattered (x,y,z) samples into a 2D depth map.

        Parameters
        ----------
        positions : (N,3) array
            columns: x, y, z (z = height/depth value)
        cell_size : float
            grid resolution in same units as x,y
        bounds : (xmin, xmax, ymin, ymax) or None
            If None, inferred from data min/max.
        agg : str
            How to combine multiple samples per cell.
        return_meta : bool
            If True, return (depth_map, meta). Otherwise return depth_map only.

        Returns
        -------
        depth_map : (H,W) array
            depth values per grid cell; empty cells are NaN.
        meta : dict
            Contains origin, cell_size, bounds, and helpers for index<->world conversion.
        """
        pos = np.asarray(positions)
        if pos.ndim != 2 or pos.shape[1] != 3:
            raise ValueError(f"positions must be (N,3), got {pos.shape}")

        x = pos[:, 0].astype(np.float64, copy=False)
        y = pos[:, 1].astype(np.float64, copy=False)
        z = pos[:, 2].astype(np.float64, copy=False)

        # Drop NaNs/Infs
        valid = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
        x, y, z = x[valid], y[valid], z[valid]
        if x.size == 0:
            raise ValueError("No valid (finite) samples in positions.")

        if bounds is None:
            xmin, xmax = float(x.min()), float(x.max())
            ymin, ymax = float(y.min()), float(y.max())
        else:
            xmin, xmax, ymin, ymax = map(float, bounds)
            if not (xmax > xmin and ymax > ymin):
                raise ValueError("bounds must satisfy xmax>xmin and ymax>ymin")

        # Define grid size. +1 to include max edge
        W = int(np.floor((xmax - xmin) / cell_size)) + 1
        H = int(np.floor((ymax - ymin) / cell_size)) + 1
        if W <= 0 or H <= 0:
            raise ValueError("Computed grid has non-positive dimensions; check bounds/cell_size.")

        # Convert world coords -> integer grid indices
        # i: row (y), j: col (x)
        j = np.floor((x - xmin) / cell_size).astype(np.int64)
        i = np.floor((y - ymin) / cell_size).astype(np.int64)

        # Keep only samples within bounds (numerical safety)
        inb = (i >= 0) & (i < H) & (j >= 0) & (j < W)
        i, j, z = i[inb], j[inb], z[inb]

        depth_map = np.full((H, W), np.nan, dtype=dtype)

        if z.size == 0:
            # No samples fell into grid (rare but possible if bounds tight)
            meta = {
                "xmin": xmin, "xmax": xmax, "ymin": ymin, "ymax": ymax,
                "cell_size": float(cell_size),
                "H": H, "W": W,
            }
            return (depth_map, meta) if return_meta else depth_map

        # --- Aggregation ---
        # We aggregate per-cell using a fast "group by" on linear indices.
        lin = i * W + j
        order = np.argsort(lin)
        lin_s = lin[order]
        z_s = z[order]

        # group boundaries
        uniq_lin, start_idx = np.unique(lin_s, return_index=True)
        # end indices are next start or end
        end_idx = np.r_[start_idx[1:], z_s.size]

        if agg == "last":
            # last sample in the sorted order for each cell (deterministic)
            z_agg = z_s[end_idx - 1]
        elif agg == "min":
            z_agg = np.minimum.reduceat(z_s, start_idx)
        elif agg == "max":
            z_agg = np.maximum.reduceat(z_s, start_idx)
        elif agg == "mean":
            sums = np.add.reduceat(z_s, start_idx)
            counts = (end_idx - start_idx).astype(np.float64)
            z_agg = sums / counts
        elif agg == "median":
            # median per group: loop over groups (usually fine; groups << N)
            z_agg = np.empty_like(uniq_lin, dtype=np.float64)
            for k, (a, b) in enumerate(zip(start_idx, end_idx)):
                z_agg[k] = np.median(z_s[a:b])
        else:
            raise ValueError("agg must be one of: 'median','mean','min','max','last'")

        # Scatter back into map
        ii = (uniq_lin // W).astype(np.int64)
        jj = (uniq_lin %  W).astype(np.int64)
        depth_map[ii, jj] = z_agg.astype(dtype, copy=False)

        meta = {
            "xmin": xmin, "xmax": xmax,
            "ymin": ymin, "ymax": ymax,
            "cell_size": float(cell_size),
            "H": H, "W": W,
            # helpers
            "grid_to_world": lambda i, j: (xmin + (j + 0.5) * cell_size,
                                          ymin + (i + 0.5) * cell_size),
            "world_to_grid": lambda xw, yw: (int(np.floor((yw - ymin) / cell_size)),
                                            int(np.floor((xw - xmin) / cell_size))),
        }
        
        nan_ratio = np.isnan(depth_map).mean()
        print("NaN ratio:", nan_ratio)

        return (depth_map, meta) if return_meta else depth_map
    
    def fill_nans_nearest(depth_map: np.ndarray) -> np.ndarray:
        d = np.asarray(depth_map, dtype=np.float32).copy()
        mask = ~np.isfinite(d)  # True where NaN/Inf

        # If everything is invalid, nothing to do
        if mask.all():
            return d

        # distance_transform_edt expects "features" as False and "background" as True for nearest-feature indices.
        # We want nearest *valid* pixels, so pass mask (True=invalid/background).
        dist, inds = ndimage.distance_transform_edt(mask, return_indices=True)

        # inds has shape (2, H, W) for 2D; tuple(inds) is (row_idx_map, col_idx_map)
        nearest = d[tuple(inds)]  # shape (H, W): value of nearest valid pixel for each location

        # Patch only invalid locations
        d[mask] = nearest[mask]
        return d

    def patch_depth(depth_map: np.ndarray, smooth_sigma: float = 0.2, d_f = 10, sigmaColor = 0.1, sigmaSpace = 1) -> np.ndarray:
        d = DepthEstimation.fill_nans_nearest(depth_map)  

        # Edge-preserving smoothing (optional; tune)
        d = cv2.bilateralFilter(d.astype(np.float32), d=d_f, sigmaColor=sigmaColor, sigmaSpace=sigmaSpace)

        # Optional final light Gaussian
        if smooth_sigma and smooth_sigma > 0:
            d = ndimage.gaussian_filter(d, smooth_sigma)

        return d

    def save_depth_npz(path, depth_map: np.ndarray, meta: dict):
        # Store only JSON-serializable meta fields (lambdas can’t be saved)
        meta_save = {k: v for k, v in meta.items() if k not in ("grid_to_world", "world_to_grid")}
        np.savez_compressed(path, depth=depth_map, meta=meta_save)

    def load_depth_npz(path):
        data = np.load(path, allow_pickle=True)
        depth = data["depth"]
        meta = data["meta"].item()
        nan_ratio = np.isnan(depth).mean()
        print("NaN ratio:", nan_ratio)
        return depth, meta
    
    def plot_depth_surface(depth_map: np.ndarray, meta: dict, title="Surface Mapping"):
        Z = np.asarray(depth_map, dtype=float)
        H, W = Z.shape
        cs = float(meta["cell_size"])

        xs = float(meta["xmin"]) + (np.arange(W) + 0.5) * cs
        ys = float(meta["ymin"]) + (np.arange(H) + 0.5) * cs
        X, Y = np.meshgrid(xs, ys)

        # Mask invalids (plot_surface handles masked arrays well)
        Zm = np.ma.masked_invalid(Z)

        fig = plt.figure()
        ax = fig.add_subplot(111, projection="3d")

        ax.plot_surface(X, Y, Zm, rstride=1, cstride=1, linewidth=0, antialiased=True)

        ax.set_title(title)
        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")
        ax.set_zlabel("z [m]")

        # Make sure the z-range isn't collapsing visually
        zmin = float(np.nanmin(0))
        zmax = float(np.nanmax(11 / 1000))
        if np.isfinite(zmin) and np.isfinite(zmax) and zmax > zmin:
            ax.set_zlim(zmin, zmax)

        plt.show()

    @staticmethod
    def current_height(
        depth_map: np.ndarray,
        current_position: np.ndarray,
        meta: Optional[Dict] = None,
        window: int = 2,
        invalid_value: float = np.nan,
    ) -> float:
        """
        Estimate current height from a depth map at a given position.

        Parameters
        ----------
        depth_map : (H,W) array
            Height/depth value per grid cell. May include NaNs.
        current_position : array-like
            If meta is provided: expects (x, y) in world units.
            If meta is None: expects (i, j) integer grid indices.
        meta : dict or None
            Must contain: xmin, ymin, cell_size.
            If provided, converts world (x,y) -> grid (i,j).
        window : int
            Neighborhood half-size (in cells) to search for a valid (non-NaN) depth.
            window=2 searches a (5x5) patch centered at (i,j).
        method : str
            "nearest_valid": return nearest valid cell in the window (by Euclidean distance in grid cells).
            "bilinear": bilinear interpolation (requires 4 neighbors valid); falls back to nearest_valid.
        invalid_value : float
            Value returned if no valid depth found (default NaN).

        Returns
        -------
        z : float
            Estimated height/depth at the current position (same units as depth_map values).
        """
        Z = np.asarray(depth_map, dtype=float)
        if Z.ndim != 2:
            raise ValueError(f"depth_map must be 2D, got shape {Z.shape}")

        H, W = Z.shape
        p = np.asarray(current_position).reshape(-1)
        if p.size < 2:
            raise ValueError("current_position must have at least 2 elements")

        # --- Convert position to grid indices ---
        if meta is not None:
            # world -> grid
            xmin = float(meta["xmin"])
            ymin = float(meta["ymin"])
            cs = float(meta["cell_size"])
            x, y = float(p[0]), float(p[1])

            j_f = (x - xmin) / cs
            i_f = (y - ymin) / cs
        else:
            # already grid indices
            i_f = float(p[0])
            j_f = float(p[1])


        # --- Nearest valid in a local window ---
        i_c = int(np.round(i_f))
        j_c = int(np.round(j_f))

        # Clamp center inside bounds
        i_c = max(0, min(H - 1, i_c))
        j_c = max(0, min(W - 1, j_c))

        # If center is valid
        if np.isfinite(Z[i_c, j_c]):
            return float(Z[i_c, j_c])

        # Search neighborhood
        i_min = max(0, i_c - window)
        i_max = min(H - 1, i_c + window)
        j_min = max(0, j_c - window)
        j_max = min(W - 1, j_c + window)

        patch = Z[i_min:i_max + 1, j_min:j_max + 1]
        valid = np.isfinite(patch)
        if not np.any(valid):
            return float(invalid_value)

        # Find nearest valid cell by grid distance
        pi, pj = np.where(valid)

        pi_full = pi + i_min
        pj_full = pj + j_min

        di = pi_full - i_f
        dj = pj_full - j_f
        k = int(np.argmin(di * di + dj * dj))

        return float(Z[pi_full[k], pj_full[k]])


def main():
    path = "surgical_system/py_src/registration/calibration_info/"
    stack_path = path + "homography_stack.npz"
    homography_stack = DepthEstimation.load_homography_stack_npz(stack_path)
    dense_stack = DepthEstimation.create_dense_stack(homography_stack)
    
    
    # z_best, err_best, uv_best, conf = DepthEstimation.estimate_depth_from_dense_stack(
    #     dense_stack,
    #     (608.52271995, 238.49790525), # reference 6.16 - 2.23 mm 
    #     (0, 0),
    #     refine=True) 
    
    # print("Actual Height [mm]: ", 6.16 - 2.23)
    # print("Predicted Z [mm]: ", z_best * 1000, "| Error: ", err_best, "| Best Prediction: ", uv_best, "| Confidence: ", conf)
    path = "surgical_system/py_src/registration/calibration_info/depth_map.npz"
    depth, meta = DepthEstimation.load_depth_npz(path)
    depth_map = DepthEstimation.patch_depth(depth)
    DepthEstimation.plot_depth_surface(depth_map, meta)
    
if __name__ == "__main__":
    main()