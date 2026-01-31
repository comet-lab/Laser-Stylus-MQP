import numpy as np
import cv2

class Display_World_Transform:
    """
    Coordinate-frame API for one camera.

    Frames:
      - img_px: original camera image pixels (native resolution)
      - warped_disp_px: pixels in the warped, shifted, flipped "displayable" canvas
      - disp_px: UI/display pixels (e.g., 1280x720 canvas the user interacts with)
      - world_m: planar world meters (x,y)

    Internals:
      - H_display maps img_px -> warped_disp_px 
      - S_disp2warp maps disp_px -> warped_disp_px
      - A_warped_disp_to_world maps disp_px -> world_m  
      - A_world_to_warped_disp maps world_m -> disp_px
    """

    def __init__(self, H, img_shape, pix_per_m, display_size):
        self.pix_per_m = float(pix_per_m)
        self.display_h, self.display_w= map(float, display_size)

        if H.shape != (3, 3):
            raise ValueError(f"H must be 3x3, got {H.shape}")
        
        self.H = np.asarray(H, dtype=np.float32)


        self._build_positive_homography(self.H, img_shape)
        self._build_display_scaling()
        self._compose_world_mappings()

    # -------------------------
    # Build steps (given)
    # -------------------------

    def _build_positive_homography(self, H, img_shape):
        h, w = img_shape[:2]

        corners = np.float32([[0, 0], [w - 1, 0], [w - 1, h - 1], [0, h - 1]]).reshape(-1, 1, 2)
        warped = cv2.perspectiveTransform(corners, H).reshape(-1, 2)

        self.x_min, self.y_min = warped.min(axis=0)
        x_max, y_max = warped.max(axis=0)

        self.out_w = int(np.ceil(x_max - self.x_min))
        self.out_h = int(np.ceil(y_max - self.y_min))

        self.T_shift = np.array([[1, 0, -self.x_min],
                                 [0, 1, -self.y_min],
                                 [0, 0, 1]], dtype=np.float32)

        self.V_flip = np.array([[1, 0, 0],
                                [0, -1, self.out_h - 1],
                                [0, 0, 1]], dtype=np.float32)

        self.H_display = self.V_flip @ self.T_shift @ H  # img_px -> warped_disp_px

    def _build_display_scaling(self):
        sx = self.out_w / self.display_w
        sy = self.out_h / self.display_h

        self.S_disp2warp = np.array([
            [sx, 0, 0],
            [0, sy, 0],
            [0, 0, 1]
        ], dtype=np.float32)

        self.S_warp2disp = np.linalg.inv(self.S_disp2warp).astype(np.float32)

    def _compose_world_mappings(self):
        ppm = float(self.pix_per_m)

        P_pix2m = np.array([[1.0 / ppm, 0, 0],
                            [0, 1.0 / ppm, 0],
                            [0, 0, 1]], dtype=np.float32)

        P_m2pix = np.array([[ppm, 0, 0],
                            [0, ppm, 0],
                            [0, 0, 1]], dtype=np.float32)
        
        # img_px -> world_m
        self.H_img_to_world = (P_pix2m @ self.H).astype(np.float32)

        # world_m -> img_px
        self.H_world_to_img = np.linalg.inv(self.H_img_to_world).astype(np.float32)
    

        V_unflip = np.linalg.inv(self.V_flip).astype(np.float32)
        T_unshift = np.linalg.inv(self.T_shift).astype(np.float32)

        
        self.A_disp_to_world = (P_pix2m @ T_unshift @ V_unflip @ self.S_disp2warp).astype(np.float32)
        self.A_world_to_disp = (self.S_warp2disp @ self.V_flip @ self.T_shift @ P_m2pix).astype(np.float32)


    # -------------------------
    # Helpers
    # -------------------------

    @staticmethod
    def _as_pts2(x) -> np.ndarray:
        """Ensure (N,2) float32."""
        pts = np.asarray(x, dtype=np.float32)
        if pts.ndim == 1:
            pts = pts[None, :]
        if pts.shape[1] < 2:
            raise ValueError(f"Expected at least 2 columns, got shape {pts.shape}")
        return pts[:, :2].copy()

    @staticmethod
    def _persp(pts2: np.ndarray, H: np.ndarray) -> np.ndarray:
        """cv2.perspectiveTransform wrapper for (N,2) -> (N,2)."""
        return cv2.perspectiveTransform(pts2.reshape(-1, 1, 2), H).reshape(-1, 2)

    # -------------------------
    # API
    # -------------------------

    # ---- Image warping for display ----

    def warp_image_for_display(self, img: np.ndarray, interpolation=cv2.INTER_LINEAR) -> np.ndarray:
        """
        Warp a camera image into the displayable warped canvas.

        Input:  img in original image pixel frame (img_px)
        Output: warped image in warped_disp_px frame with size (out_w, out_h)
        """
        return cv2.warpPerspective(img, self.H_display, (self.out_w, self.out_h), flags=interpolation)

    # def img_px_to_warped_disp_px(self, img_pts) -> np.ndarray:
    #     """
    #     Map points in original image pixels -> warped display pixels.
    #     """
    #     pts = self._as_pts2(img_pts)
    #     return self._persp(pts, self.H_display)

    # def warped_disp_px_to_img_px(self, warped_pts) -> np.ndarray:
    #     """
    #     Inverse mapping: warped display pixels -> original image pixels.
    #     """
    #     pts = self._as_pts2(warped_pts)
    #     Hinv = np.linalg.inv(self.H_display).astype(np.float32)
    #     return self._persp(pts, Hinv)

    # ---- Display <-> warped-display scaling ----

    # def disp_px_to_warped_disp_px(self, disp_pts) -> np.ndarray:
    #     """
    #     UI/display pixels -> warped display pixels (scaled to match out_w/out_h).
    #     """
    #     pts = self._as_pts2(disp_pts)
    #     return self._persp(pts, self.S_disp2warp)

    # def warped_disp_px_to_disp_px(self, warped_pts) -> np.ndarray:
    #     """
    #     Warped display pixels -> UI/display pixels.
    #     """
    #     pts = self._as_pts2(warped_pts)
    #     return self._persp(pts, self.S_warp2disp)

    # ---- Display pixels <-> world meters ----

    def disp_px_to_world_m(self, disp_pts, z: float = 0.0) -> np.ndarray:
        """
        UI/display pixels -> world meters (planar), returning (N,3) with z filled.
        """
        pts = self._as_pts2(disp_pts)
        xy = self._persp(pts, self.A_disp_to_world)

        out = np.zeros((xy.shape[0], 3), dtype=np.float32)
        out[:, :2] = xy
        out[:, 2] = float(z)
        return out

    def world_m_to_disp_px(self, world_pts) -> np.ndarray:
        """
        World meters (N,2 or N,3) -> UI/display pixels (N,2).
        """
        wp = np.asarray(world_pts, dtype=np.float32)
        if wp.ndim == 1:
            wp = wp[None, :]
        pts = wp[:, :2].copy()
        return self._persp(pts, self.A_world_to_disp)

    # ---- original image px <-> world meters ----

    def img_px_to_world_m(self, img_pts, z: float = 0.0) -> np.ndarray:
        """
        Original image pixels -> world meters.

        """
        pts = np.asarray(img_pts, dtype=np.float32)
        
        world_point = np.zeros((pts.shape[0], 3))
        world_point[:, -1] = z
        world_point[:, :2] = self._persp(pts, self.H_img_to_world)
        return world_point

    def world_m_to_img_px(self, world_pts) -> np.ndarray:
        """
        World meters -> original image pixels.
        """
        wp = np.asarray(world_pts, dtype=np.float32)
        
        if wp.ndim == 1:
            wp = wp[None, :]
        pts = wp[:, :2].copy()
        return self._persp(pts, self.H_world_to_img)

    # ---- Debug ----

    def roundtrip_error_disp(self, disp_pts) -> float:
        """
        Max absolute pixel error for disp -> world -> disp roundtrip.
        """
        p = self._as_pts2(disp_pts)
        w = self.disp_px_to_world_m(p)[:, :2]
        p2 = self.world_m_to_disp_px(w)
        return float(np.max(np.abs(p2 - p)))

    def summary(self) -> dict:
        """
        Useful for logging / debugging.
        """
        return {
            "pix_per_m": self.pix_per_m,
            "display_size": (self.display_w, self.display_h),
            "out_size": (self.out_w, self.out_h),
            "min_xy": (float(self.x_min), float(self.y_min)),
            "H_display": self.H_display.copy(),
            "A_disp_to_world": self.A_disp_to_world.copy(),
            "A_world_to_disp": self.A_world_to_disp.copy(),
        }
