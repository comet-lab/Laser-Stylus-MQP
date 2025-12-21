from PIL import Image
import numpy as np
import cv2
import matplotlib.pyplot as plt

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
        return out
    
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
    
def main():
    img_path = "backend/src/saved_masks/path.png"
    img = np.array(Image.open(img_path))
    img = Motion_Planner.fill_in_shape(img)
    path = Motion_Planner.raster_pattern(img)
    # 4) Visualize the path overlayed on the mask
    fig, ax = plt.subplots(figsize=(8,4))
    ax.imshow(img, cmap='gray')
    if len(path) > 1:
        xs = [p[0] for p in path]
        ys_plot = [p[1] for p in path]
        ax.plot(xs, ys_plot, linewidth=1)  # default color
    ax.set_title("Continuous boustrophedon path inside oval")
    ax.set_axis_off()
    plt.show()
    pass

if __name__=='__main__':
    main()
    