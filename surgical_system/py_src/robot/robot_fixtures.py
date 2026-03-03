import numpy as np 
from typing import Dict, Any, Tuple
from dataclasses import dataclass
import sys, pathlib
HERE = pathlib.Path(__file__).resolve().parent
for candidate in (HERE, HERE.parent, HERE.parent.parent):
    if (candidate / "robot").is_dir() and (candidate / "cameras").is_dir() and (candidate / "laser_control").is_dir():
        sys.path.insert(0, str(candidate))
        break
from common.geometry import polygon_to_grid_map
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches

@dataclass(frozen=True)
class GridBoundary:
    valid_map: np.ndarray        # (H,W) bool or {0,1}
    grid_size: float             # meters per cell
    origin: Tuple[float, float]  # world coord of cell (0,0) corner
    
class RobotFixtures:
    def __init__(self, pts_xy: np.ndarray = np.zeros((4,2)),
                        square_size: float = 0.0005,
                        padding: float = 0.0,
                        include_boundary: bool = True,
                        boundary: Dict[str, Any] = None):
        
        if boundary is None:
            print("[Robot Fixtures] Generating mapping")
            boundary = polygon_to_grid_map(pts_xy = pts_xy,
                                           square_size = square_size,
                                           padding=padding,
                                           include_boundary=include_boundary)
            
            
            
        valid_map = np.asarray(boundary["map"])
        if valid_map.ndim != 2:
            raise ValueError("boundary['map'] must be a 2D array (H,W).")
        self.boundary = GridBoundary(
            valid_map=valid_map.astype(bool),
            grid_size=float(boundary["size"]),
            origin=tuple(boundary["origin"]),
        )
        self.H, self.W = self.boundary.valid_map.shape
    
     # ---------- world <-> grid helpers ----------
    def _world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        ox, oy = self.boundary.origin
        s = self.boundary.grid_size

        col = int(np.floor((x - ox) / s))
        row = int(np.floor((y - oy) / s))
        return row, col

    def _in_grid(self, row: int, col: int) -> bool:
        return (0 <= row < self.H) and (0 <= col < self.W)
        
    # ---------- API ----------
    def is_valid(self, position: np.ndarray) -> bool:
        p = np.asarray(position, dtype=float).reshape(-1)
        if p.size < 2:
            raise ValueError("position must contain at least (x,y).")

        row, col = self._world_to_grid(p[0], p[1])
        if not self._in_grid(row, col):
            return False
        return bool(self.boundary.valid_map[row, col])
    
    def plot_valid_region(self, ax=None):
        """
        Visualize valid fixture region in WORLD coordinates.
        """

        if ax is None:
            fig, ax = plt.subplots()

        valid = self.boundary.valid_map
        s = self.boundary.grid_size
        ox, oy = self.boundary.origin

        H, W = valid.shape

        # ---- compute world extents ----
        xmin = ox
        xmax = ox + W * s
        ymin = oy
        ymax = oy + H * s

        # imshow expects [xmin,xmax,ymin,ymax]
        ax.imshow(
            valid,
            origin="lower",          # IMPORTANT (robot frame)
            extent=[xmin, xmax, ymin, ymax],
            interpolation="nearest",
            alpha=0.7,
        )
        
        invalid_patch = mpatches.Patch(color=plt.cm.viridis(1.0),
                                label="Valid (robot allowed)")
        
        valid_patch = mpatches.Patch(color=plt.cm.viridis(0.0),
                                label="Invalid (blocked)")

        ax.legend(handles=[valid_patch, invalid_patch], loc="upper left")
    

        ax.set_aspect("equal")
        ax.set_xlabel("X [m]")
        ax.set_ylabel("Y [m]")
        ax.set_title("Robot Virtual Fixture (Valid Region)")
        ax.grid(True, alpha=0.3)
        # plt.show()
        fig.savefig("plots/robot fixtures.png")
        plt.close()

        return ax
    
        
    
    
if __name__ == "__main__":
    boundary = [[-0.0215, -0.0215],
                        [ 0.0215, -0.0215],
                        [ 0.0215,  0.0215],
                        [-0.0215,  0.0215]] 
            
    robot_fixtures = RobotFixtures(boundary, include_boundary=False)
    print(robot_fixtures.is_valid(np.array([0, 0])))
    robot_fixtures.plot_valid_region() # Debug 
