from __future__ import annotations

import pyslm
import pyslm.visualise
from pyslm import hatching as hatching
import cv2
from PIL import Image
import numpy as np
import matplotlib.pyplot as plt

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
    
def contour_to_loop(cnt, close=True, simplify_frac=0.002):
    """
    cnt: OpenCV contour (N,1,2)
    simplify_frac: fraction of perimeter for approxPolyDP (0 disables simplification)
    returns: (M,2) float array
    """
    if simplify_frac and simplify_frac > 0:
        perim = cv2.arcLength(cnt, True)
        eps = simplify_frac * perim
        cnt = cv2.approxPolyDP(cnt, eps, True)

    pts = cnt[:, 0, :].astype(np.float64)

    # remove consecutive duplicates
    if len(pts) > 1:
        keep = np.ones(len(pts), dtype=bool)
        keep[1:] = np.any(pts[1:] != pts[:-1], axis=1)
        pts = pts[keep]

    # close loop
    if close and len(pts) >= 2 and not np.allclose(pts[0], pts[-1]):
        pts = np.vstack([pts, pts[0]])

    return pts


# # Imports the part and sets the geometry to an STL file (frameGuide.stl)
# solidPart = pyslm.Part('myFrameGuide')
# solidPart.setGeometry('../models/frameGuide.stl')

# Create a StripeHatcher object for performing any hatching operations
myHatcher = hatching.StripeHatcher()
myHatcher.stripeWidth = 5.0 # [mm]

# Set the base hatching parameters which are generated within Hatcher
myHatcher.hatchAngle = 10 # [°]
myHatcher.volumeOffsetHatch = 0.08 # [mm]
myHatcher.spotCompensation = 0.06 # [mm]
myHatcher.numInnerContours = 2
myHatcher.numOuterContours = 1

# # Slice the object at Z and get the boundaries
# geomSlice = solidPart.getVectorSlice(z)

img_path = "surgical_system/py_src/motion_planning/path2.png"
img = np.array(Image.open(img_path))
img = fill_in_shape(img)

contours, _ = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        
edge = max(contours, key=cv2.contourArea)
edge = contour_to_loop(edge)
perimeter_pts = edge[:, 0, :] 
plt.plot(perimeter_pts[:, 0], perimeter_pts[:, 1])
plt.show()


# Perform the hatching operations
layer = myHatcher.hatch(perimeter_pts)

# Plot the layer geometries generated
pyslm.visualise.plot(layer, plot3D=False, plotOrderLine=True) # plotArrows=True)