import numpy as np
import cv2
from typing import Dict, Tuple, Optional
import matplotlib.pyplot as plt
from scipy import ndimage
class DepthEstimation():
    @staticmethod
    def project_xy(H: np.ndarray,
                     pts: np.ndarray,
                     eps: float = 1e-12) -> np.ndarray:
        """
        Vectorized homography projection.

        Parameters
        ----------
        H   : (3,3) homography matrix
        pts : (N,2) array of (X,Y) plane coordinates

        Returns
        -------
        uv  : (N,2) projected pixel coordinates
            invalid points (small denom) -> [inf, inf]
        """

        pts = np.asarray(pts, dtype=float)

        # Add homogeneous coordinate
        ones = np.ones((pts.shape[0], 1))
        pts_h = np.hstack((pts, ones))  # (N,3)

        # Apply homography
        p = pts_h @ H.T  # (N,3)

        denom = p[:, 2:3]                 # (N,1)
        valid = np.abs(denom) >= eps      # (N,1)

        uv = np.full((pts.shape[0], 2), np.inf)

        m = valid[:, 0]                   # (N,)
        uv[m] = p[m, :2] / denom[m, :] 

        return uv

    @staticmethod
    def estimate_depth_from_dense_stack(
        dense_stack: Dict[float, np.ndarray],
        obs_uv: Tuple[float, float],
        cmd_XY: Tuple[float, float],
        *,
        metric: str = "l2",
        refine: bool = True,
        min_valid_heights: int = 3,
    ) -> Tuple[float, float, Tuple[float, float], float]:
        """
        Estimate depth (height z) using a dense homography stack.

        Parameters
        ----------
        dense_stack : dict
            {z: H_z} where H_z is 3x3 (ideally normalized) homography at height z.
            z units(e.g., mm).
        obs_uv : (u_obs, v_obs)
            Observed pixel location of the laser spot (or feature) in the RGB image.
        cmd_XY : (X_cmd, Y_cmd)
            Commanded / known plane coordinates (object coordinates in the calibration plane frame).
        metric : "l2" or "l1"
            Error metric in pixel space.
        refine : bool
            If True, do a local quadratic refinement around the best discrete z (sub-grid estimate).
        min_valid_heights : int
            Require at least this many heights in the stack.

        Returns
        -------
        z_hat : float
            Estimated depth/height.
        err_hat : float
            Residual error at z_hat (pixels, metric-dependent).
        uv_pred : (u_pred, v_pred)
            Predicted pixel at z_hat.
        conf : float
            Simple confidence proxy in (0,1], based on error separation between best and 2nd best.
            (Higher is better.) Use your own gating thresholds in practice.

        Notes
        -----
        - This assumes the environment can be approximated by one height z for this point.
        """
        if len(dense_stack) < min_valid_heights:
            raise ValueError(f"dense_stack must have at least {min_valid_heights} entries.")

        u_obs, v_obs = map(float, obs_uv)
        X_cmd, Y_cmd = map(float, cmd_XY)

        # Sort heights
        zs = np.array(sorted(dense_stack.keys()), dtype=float)
        # Hs = np.array([dense_stack[z] for z in zs])
        Hs = np.array([np.linalg.inv(dense_stack[z]) for z in zs])

        # ----- Project commanded point through all homographies -----

        # Numerators
        u_num = Hs[:, 0, 0] * X_cmd + Hs[:, 0, 1] * Y_cmd + Hs[:, 0, 2]
        v_num = Hs[:, 1, 0] * X_cmd + Hs[:, 1, 1] * Y_cmd + Hs[:, 1, 2]
        denom = Hs[:, 2, 0] * X_cmd + Hs[:, 2, 1] * Y_cmd + Hs[:, 2, 2]

        eps = 1e-12
        valid = np.abs(denom) >= eps

        # Initialize predictions
        preds = np.full((Hs.shape[0], 2), np.inf)

        preds[valid, 0] = u_num[valid] / denom[valid]
        preds[valid, 1] = v_num[valid] / denom[valid]

        # ----- Compute residuals -----
        # print("Predictions: ", preds)
        du = preds[:, 0] - u_obs
        dv = preds[:, 1] - v_obs

        if metric == "l2":
            errs = np.hypot(du, dv)
        elif metric == "l1":
            errs = np.abs(du) + np.abs(dv)
        else:
            raise ValueError("metric must be 'l2' or 'l1'")

        # ----- Select best depth -----

        best_idx = int(np.argmin(errs))
        # print("predicted heights :", zs)
        # print("errors: :", errs)
        z_best = float(zs[best_idx])
        err_best = float(errs[best_idx])
        uv_best = (float(preds[best_idx, 0]), float(preds[best_idx, 1]))

        # Confidence proxy
        if zs.size >= 2:
            sorted_errs = np.sort(errs)
            err2 = float(sorted_errs[1])
            conf = float(np.clip((err2 - err_best) / max(err2, 1e-9), 0.0, 1.0))
        else:
            conf = 0.0

        if not refine or best_idx == 0 or best_idx == zs.size - 1:
            return z_best, err_best, uv_best, conf

        # ---- Local quadratic refinement on error(z) using 3 neighboring samples ----
        # Fit e(z) ≈ a z^2 + b z + c around best_idx-1, best_idx, best_idx+1
        z0, z1, z2 = float(zs[best_idx - 1]), float(zs[best_idx]), float(zs[best_idx + 1])
        e0, e1, e2 = float(errs[best_idx - 1]), float(errs[best_idx]), float(errs[best_idx + 1])

        # Solve for parabola coefficients via Lagrange form (stable for 3 points)
        d01 = z0 - z1
        d02 = z0 - z2
        d12 = z1 - z2

        a = (e0 / (d01 * d02)
            - e1 / (d01 * d12)
            + e2 / (d02 * d12))

        b = (-e0 * (z1 + z2) / (d01 * d02)
            + e1 * (z0 + z2) / (d01 * d12)
            - e2 * (z0 + z1) / (d02 * d12))

        if abs(a) < 1e-12:
            # Nearly linear; refinement not meaningful
            return z_best, err_best, uv_best, conf
 
        z_ref = -b / (2.0 * a)

        # Clamp refined z to the local interval to avoid nonsense
        z_ref = float(np.clip(z_ref, min(z0, z2), max(z0, z2)))

        # Interpolate homography between nearest neighbors for refined z (piecewise linear)
        if z_ref <= z1:
            # between z0 and z1
            t = 0.0 if z1 == z0 else (z_ref - z0) / (z1 - z0)
            H_ref = (1.0 - t) * np.asarray(dense_stack[z0], float) + t * np.asarray(dense_stack[z1], float)
        else:
            # between z1 and z2
            t = 0.0 if z2 == z1 else (z_ref - z1) / (z2 - z1)
            H_ref = (1.0 - t) * np.asarray(dense_stack[z1], float) + t * np.asarray(dense_stack[z2], float)

        uv_ref = DepthEstimation.project_xy(H_ref, np.array([[X_cmd, Y_cmd]]))
        u_ref, v_ref = uv_ref[0]
        
        du = u_ref - u_obs
        dv = v_ref - v_obs
        err_ref = float(np.hypot(du, dv)) if metric == "l2" else float(abs(du) + abs(dv))

        # Keep refinement only if it improves
        if np.isfinite(err_ref) and err_ref <= err_best:
            return z_ref, err_ref, (float(u_ref), float(v_ref)), conf

        return z_best, err_best, uv_best, conf
    
    @staticmethod
    def load_homography_stack_npz(file_path):
        data = np.load(file_path)

        zs = data["zs"]       # (N,)
        Hs = data["Hs"]       # (N,3,3)

        return {float(z): H for z, H in zip(zs, Hs)}
    
    @staticmethod
    def normalize_homography(H, eps=1e-9):
        """
        Normalize homography to remove scale ambiguity.
        Prefer H[2,2] normalization; fallback to Frobenius.
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

    def patch_depth(depth_map: np.ndarray, smooth_sigma: float = 0.5) -> np.ndarray:
        d = DepthEstimation.fill_nans_nearest(depth_map)  

        # Edge-preserving smoothing (optional; tune)
        d = cv2.bilateralFilter(d.astype(np.float32), d=10, sigmaColor=0.003, sigmaSpace=1)

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
    
    def plot_depth_surface(depth_map: np.ndarray, meta: dict, title="Depth surface"):
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
        ax.set_xlabel("x")
        ax.set_ylabel("y")
        ax.set_zlabel("depth / height")

        # Make sure the z-range isn't collapsing visually
        zmin = float(np.nanmin(0))
        zmax = float(np.nanmax(11 / 1000))
        if np.isfinite(zmin) and np.isfinite(zmax) and zmax > zmin:
            ax.set_zlim(zmin, zmax)

        plt.show()




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