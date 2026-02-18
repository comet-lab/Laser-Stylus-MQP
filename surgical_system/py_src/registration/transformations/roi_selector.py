import cv2
import numpy as np

WINDOW_NAME = "Adjustable ROI"
ROI_WINDOW = "ROI View"

class ROISelector:
    def __init__(self, img):
        self.img = img
        self.clone = img.copy()

        h, w = img.shape[:2]
        # Start with a centered square
        size = min(w, h) // 3
        cx, cy = w // 2, h // 2
        self.points = [
            [cx - size // 2, cy - size // 2],  # top-left
            [cx + size // 2, cy - size // 2],  # top-right
            [cx + size // 2, cy + size // 2],  # bottom-right
            [cx - size // 2, cy + size // 2],  # bottom-left
        ]

        self.dragging_idx = None
        self.handle_radius = 10  # pixel distance to detect corner grab

    def draw(self, img):
        """Draw the rectangle and the corner handles on img."""
        # Draw polygon
        pts = np.array(self.points, dtype=np.int32)
        cv2.polylines(img, [pts], isClosed=True, color=(0, 255, 0), thickness=2)

        # Draw corner handles
        for (x, y) in self.points:
            cv2.rectangle(img,
                          (x - 5, y - 5),
                          (x + 5, y + 5),
                          (0, 0, 255),
                          thickness=-1)

    def get_roi(self):
        """Return the current axis-aligned bounding box ROI as an image."""
        xs = [p[0] for p in self.points]
        ys = [p[1] for p in self.points]
        x_min, x_max = max(min(xs), 0), min(max(xs), self.img.shape[1])
        y_min, y_max = max(min(ys), 0), min(max(ys), self.img.shape[0])

        if x_max > x_min and y_max > y_min:
            return self.img[y_min:y_max, x_min:x_max]
        else:
            return None

    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            # Check if mouse is near a corner
            for i, (px, py) in enumerate(self.points):
                if abs(x - px) <= self.handle_radius and abs(y - py) <= self.handle_radius:
                    self.dragging_idx = i
                    break

        elif event == cv2.EVENT_MOUSEMOVE and self.dragging_idx is not None:
            # Drag selected corner
            self.points[self.dragging_idx] = [x, y]

        elif event == cv2.EVENT_LBUTTONUP:
            self.dragging_idx = None


def main():
    # Example image: replace this with your own image, or use a blank canvas
    h, w = 480, 640
    img = np.full((h, w, 3), 40, dtype=np.uint8)  # dark gray background
    cv2.putText(img, "Drag red squares to adjust ROI",
                (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (200, 200, 200), 2)

    selector = ROISelector(img)

    cv2.namedWindow(WINDOW_NAME)
    cv2.setMouseCallback(WINDOW_NAME, selector.mouse_callback)

    while True:
        frame = selector.img.copy()
        selector.draw(frame)

        cv2.imshow(WINDOW_NAME, frame)

        # Show the ROI in a separate window
        roi = selector.get_roi()
        if roi is not None and roi.size > 0:
            cv2.imshow(ROI_WINDOW, roi)

        key = cv2.waitKey(20) & 0xFF
        if key == 27:  # ESC to quit
            break

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
