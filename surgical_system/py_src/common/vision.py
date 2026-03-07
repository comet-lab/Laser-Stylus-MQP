import numpy as np
import cv2 


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
        
