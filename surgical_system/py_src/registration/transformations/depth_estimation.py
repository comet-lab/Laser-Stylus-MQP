import numpy as np
import cv2
from typing import Dict, Tuple, Optional

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

        denom = p[:, 2:3]  # (N,1)

        # Avoid divide-by-zero
        valid = np.abs(denom) >= eps

        uv = np.full((pts.shape[0], 2), np.inf)

        uv[valid[:, 0]] = p[valid[:, 0], :2] / denom[valid]

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
        - For tilt-aware estimation, you'd solve (z0,a,b) over multiple probes instead.
        """
        if len(dense_stack) < min_valid_heights:
            raise ValueError(f"dense_stack must have at least {min_valid_heights} entries.")

        u_obs, v_obs = map(float, obs_uv)
        X_cmd, Y_cmd = map(float, cmd_XY)

        # Sort heights
        zs = np.array(sorted(dense_stack.keys()), dtype=float)
        Hs = np.array([dense_stack[z] for z in zs])

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
        print("Predictions: ", preds)
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
        print("predicted heights :", zs)
        print("errors: :", errs)
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
                        dz: float = 0.0005, # m
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
    

def main():
    path = "surgical_system/py_src/registration/calibration_info/"
    stack_path = path + "homography_stack.npz"
    homography_stack = DepthEstimation.load_homography_stack_npz(stack_path)
    dense_stack = DepthEstimation.create_dense_stack(homography_stack)
    
    
    z_best, err_best, uv_best, conf = DepthEstimation.estimate_depth_from_dense_stack(
        dense_stack,
        (608.52271995, 238.49790525), # reference 6.16 - 2.23 mm 
        (0, 0),
        refine=True) 
    
    print(z_best, err_best, uv_best, conf)
    
if __name__ == "__main__":
    main()