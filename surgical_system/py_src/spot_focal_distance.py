# Import statements
import numpy as np
import time
import cv2
from laser_control.laser_arduino import Laser_Arduino
from robot.robot_controller import Robot_Controller
from cameras.RGBD_cam import RGBD_Cam
from registration.cameraRegistration import Camera_Registration
import matplotlib.pyplot as plt



def setup():
    ##################################################################################
    #-------------------------------- Laser Config ----------------------------------#
    ##################################################################################
    
    laser_controller = Laser_Arduino()  # controls whether laser is on or off
    laser_on = False
    laser_controller.vf_valid_flag = True
    laser_controller.set_output(laser_on)
    
    ##################################################################################
    #------------------------------ Robot Config ------------------------------------#
    ##################################################################################
    # Create FrankaNode object for controlling robot
    robot_controller = Robot_Controller(laser_controller)
    # TODO fix running in container
    home_pose = robot_controller.load_home_pose()
    # home_pose = robot_controller.load_edit_pose()
    start_pos = np.array([0,0,0.10]) # [m,m,m]
    target_pose = np.array([[1.0, 0, 0, start_pos[0]],
                            [0,1,0,start_pos[1]],
                            [0,0,1,start_pos[2]],
                            [0,0,0,1]])
    robot_controller.go_to_pose(target_pose@home_pose,1) # Send robot to start position
    time.sleep(2)
    

    # FIMXE: Remove thermal camera dependence
    ##################################################################################
    #--------------------------- Thermal Cam Config ---------------------------------#
    ##################################################################################
    # Set up camera object
    window_scale = 1
    frame_rate = 50  
    temp_scale = 100.0  # based on temperature linear 10mK reading
    # start with full window so we can perform camera calibration. Additionally, set maximum frame rate at 50 hz, 
    # and the focal distance to 0.204 m. This seems to be at the right location to maximize the focal point around the 
    # free beam laser spot.
    therm_cam = None
    # TODO: Uncomment me for thermal camera
    #therm_cam = ThermalCam(IRFormat="TemperatureLinear10mK", height=int(480/window_scale),frame_rate="Rate50Hz",focal_distance=0.2) 
    
    ##################################################################################
    #------------------------------ RGBD Cam Config ---------------------------------#
    ##################################################################################
    rgbd_cam = RGBD_Cam() #Runs a thread internally
    rgbd_cam.display_color_stream()
    
    ### -------------------- Run calibration ---------------- ####
    # # # Create camera registration object to perform calibration and surface scan
    camera_reg = Camera_Registration(therm_cam,rgbd_cam, robot_controller, laser_controller)

    # Define home pose of robot
    camera_reg.robot_controller.load_edit_pose()

    return camera_reg

def calc_traj(camera_reg):
     # Create grid of 9x9 points on 50 x 50 mm block
        gridSize = np.array([9, 9])
        xPoints = (np.arange(gridSize[0]) - (gridSize[0] - 1) / 2) * 0.005
        yPoints = (np.arange(gridSize[1]) - (gridSize[1] - 1) / 2) * 0.005
        xValues, yValues = np.meshgrid(xPoints, yPoints)
    
        # Calculate the trajectory (moving downwards in z direction)
        # FIXME: Starting point is negative relative to home position csv for scanning
        numPoints = gridSize[0] * gridSize[1]
        heights = np.linspace(-0.06, 0.02, numPoints) #m
        robot_path = np.hstack((xValues.reshape((-1, 1)), 
                                        yValues.reshape((-1, 1)), 
                                        heights.reshape((-1, 1))))
        return robot_path, heights

def run_traj(camera_reg, robot_path, heights):
    # Send robot to starting position (first point in trajectory)
        input(f"Press Enter to begin focal distance test.")
        start_pos = robot_path[0, :]
        start_pose = np.eye(4)
        start_pose[:3, -1] = start_pos
        print("Heading to starting location")
        camera_reg.robot_controller.go_to_pose(start_pose @ camera_reg.robot_controller.home_pose)
    
        # Loop to calculate diameters at different heights and record them
        diameters = np.zeros(len(heights))
        for i in range(robot_path.shape[0]):
            print("Heading to height: ", heights[i])
            # Send robot/laser to predetermined point
            robot_pos = robot_path[i,:]
            robot_pose = np.eye(4)
            robot_pose[:3,-1] = robot_path[i, :]
            camera_reg.robot_controller.go_to_pose(robot_pose @ camera_reg.robot_controller.home_pose)
    
            # Fire the laser (1/4 second)
            duration = 0.25
            camera_reg.fireLaser(duration)
            
            # TODO/FIXME: Measure diameters of laser's aiming beam
            # # Get RGB camera image 
            # color_img = camera_reg.rgbd_cam.get_latest()['color']
            # # Determine diameter of laser spot
            # diameter = get_spot_diameter(color_img)
            # diameters[i] = diameter

def get_spot_diameter(img):
    # Reduces noise with gaussian filter; blurs image slightly
    img_blur = cv2.GaussianBlur(img, (5, 5), 0)

    # Converts image from BGR to HSV color space
    hsv = cv2.cvtColor(img_blur, cv2.COLOR_BGR2HSV)

    # Keeps only bright pixels and everything else becomes black
    mask = cv2.inRange(hsv, (0, 0, 200), (179, 60, 255))

    # Finds all separate blobs in the binary image
    num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(mask)

    # Returns nothing if no blob is found
    if num_labels <= 1:
        return np.array([np.nan, np.nan])

    # Creates a list of blob IDs
    valid = np.arange(1, num_labels)

    # Finds blob with biggest area
    largest_blob = max(valid, key=lambda i: stats[i, cv2.CC_STAT_AREA])

    # Center coordinates
    cx, cy = centroids[largest_blob]

    # Blob area in pixels
    area = stats[largest_blob, cv2.CC_STAT_AREA]

    # Equivalent circular diameter in pixels
    diameter = 2 * np.sqrt(area / np.pi)
    return diameter

# def measure_burn_diameter(color_img):

    h, w = color_img.shape[:2]

    # # FIXME: Crop relative to current image
    # crop_w = 400
    # crop_h = 300

    # x1 = (w - crop_w) // 2
    # y1 = (h - crop_h) // 2

    # color_img = color_img[
    #     y1:y1 + crop_h,
    #     x1:x1 + crop_w
    # ]

    # Convert to grayscale
    gray = cv2.cvtColor(color_img, cv2.COLOR_BGR2GRAY)

    # Smooth slightly
    gray = cv2.GaussianBlur(gray, (5, 5), 0)

    # Create binary image
    binary = cv2.adaptiveThreshold(
        gray,
        255,
        cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
        cv2.THRESH_BINARY_INV,
        31,
        15
    )

    # Find connected white regions
    num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(
        binary,
        connectivity=8
    )

    objects = []

    # Label 0 is the black background, so skip it
    for i in range(1, num_labels):

        area = stats[i, cv2.CC_STAT_AREA]

        x = stats[i, cv2.CC_STAT_LEFT]
        y = stats[i, cv2.CC_STAT_TOP]
        w = stats[i, cv2.CC_STAT_WIDTH]
        h = stats[i, cv2.CC_STAT_HEIGHT]

        cx, cy = centroids[i]

        # Ignore objects that are obviously too small/large
        if 10 < area < 60:

            objects.append({
                "area": area,
                "center": (cx, cy),
                "bbox": (x, y, w, h)
            })

    print("Number of white objects:", len(objects))

    if not objects:
        return None

    # Find the smallest object by area
    smallest = min(objects, key=lambda obj: obj["area"])

    print("Smallest area:", smallest["area"])
    print("Smallest center:", smallest["center"])
    print("Smallest bbox:", smallest["bbox"])

    # --------------------------------------------------
    # Highlight smallest object
    # --------------------------------------------------

    debug = color_img.copy()

    cx, cy = smallest["center"]
    x, y, w, h = smallest["bbox"]

    # Draw bounding box
    cv2.rectangle(
        debug,
        (x, y),
        (x + w, y + h),
        (0, 0, 255),
        2
    )

    # Draw center point
    cv2.circle(
        debug,
        (int(cx), int(cy)),
        4,
        (0, 255, 0),
        -1
    )

    # Display area
    cv2.putText(
        debug,
        f"Area: {smallest['area']}",
        (x, max(y - 10, 20)),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5,
        (0, 0, 255),
        1
    )

    # Display image
    cv2.imshow("Smallest Ring", debug)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    # Print all areas
    print("Areas:")
    for i, obj in enumerate(objects):
        print(i + 1, obj["area"])

def measure_burn_diameter(image, pixels_per_mm=None, min_spot_area=15,
                       max_spot_area_ratio=0.05, show=True):
    """
    Analyze an image of thermal paper containing a grid of burn marks and
    find the burn mark with the smallest diameter.
 
    Parameters
    ----------
    image : str or np.ndarray
        Path to an image file, or an already-loaded BGR image (e.g. from
        cv2.imread).
    pixels_per_mm : float, optional
        If provided, diameters are also reported in millimeters.
    min_spot_area : int
        Minimum contour area (in pixels, on the straightened paper) for a
        blob to be considered a real burn mark. Filters out noise.
    max_spot_area_ratio : float
        Maximum area of a burn-mark contour, as a fraction of the paper's
        total area. Filters out anything that's clearly not a single spot.
    show : bool
        If True, pop up a window with the original image overlaid with all
        detected spots (green) and an arrow + red circle on the smallest one.
 
    Returns
    -------
    dict:
        'diameter_px' : float, diameter of the smallest spot in pixels
        'diameter_mm' : float or None, diameter in mm if pixels_per_mm given
        'center'      : (x, y) center of the smallest spot in ORIGINAL image
                        coordinates
        'all_spots'   : list of {'center_warped': (x,y), 'diameter_px': d}
                        for every detected spot (coordinates on the
                        straightened paper image)
    """
 
    # ---- 1. Load image -------------------------------------------------
    if isinstance(image, str):
        img = cv2.imread(image)
        if img is None:
            raise FileNotFoundError(f"Could not read image at {image}")
    else:
        img = image.copy()
 
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
 
    # ---- 2. Find the white thermal-paper rectangle ----------------------
    _, paper_mask = cv2.threshold(
        blurred, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU
    )
 
    contours, _ = cv2.findContours(
        paper_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )
    if not contours:
        raise RuntimeError("Could not find any bright region (thermal paper) in the image.")
 
    paper_contour = max(contours, key=cv2.contourArea)
    rect = cv2.minAreaRect(paper_contour)
    box = cv2.boxPoints(rect)
    box = np.intp(box)
 
    # ---- 3. Warp the paper to a straightened, top-down view -------------
    width = int(rect[1][0])
    height = int(rect[1][1])
    if width == 0 or height == 0:
        raise RuntimeError("Detected paper region has zero area.")
 
    src_pts = _order_points(box.astype("float32"))
    dst_pts = np.array(
        [[0, height - 1], [0, 0], [width - 1, 0], [width - 1, height - 1]],
        dtype="float32",
    )
 
    M = cv2.getPerspectiveTransform(src_pts, dst_pts)
    warped = cv2.warpPerspective(img, M, (width, height))
    warped_gray = cv2.cvtColor(warped, cv2.COLOR_BGR2GRAY)
 
    # ---- 4. Find burn marks within the paper -----------------------------
    warped_blur = cv2.GaussianBlur(warped_gray, (5, 5), 0)
    _, spots_mask = cv2.threshold(
        warped_blur, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU
    )
 
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    spots_mask = cv2.morphologyEx(spots_mask, cv2.MORPH_OPEN, kernel, iterations=1)
 
    spot_contours, _ = cv2.findContours(
        spots_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )
 
    paper_area = width * height
    max_spot_area = paper_area * max_spot_area_ratio
 
    spots = []
    for c in spot_contours:
        area = cv2.contourArea(c)
        if area < min_spot_area or area > max_spot_area:
            continue
        diameter = 2 * np.sqrt(area / np.pi)  # equivalent diameter
        (cx, cy), _ = cv2.minEnclosingCircle(c)
        spots.append({
            "contour": c,
            "center_warped": (cx, cy),
            "diameter_px": diameter,
        })
 
    if not spots:
        raise RuntimeError(
            "No burn marks were detected. Try adjusting min_spot_area / "
            "max_spot_area_ratio."
        )
 
    # ---- 5. Find the smallest spot ---------------------------------------
    smallest = min(spots, key=lambda s: s["diameter_px"])
 
    M_inv = np.linalg.inv(M)
    pt = np.array([[smallest["center_warped"]]], dtype="float32")
    pt_orig = cv2.perspectiveTransform(pt, M_inv)[0][0]
    smallest_center_orig = (float(pt_orig[0]), float(pt_orig[1]))
 
    diameter_mm = None
    if pixels_per_mm:
        diameter_mm = smallest["diameter_px"] / pixels_per_mm
 
    result = {
        "diameter_px": smallest["diameter_px"],
        "diameter_mm": diameter_mm,
        "center": smallest_center_orig,
        "all_spots": [
            {"center_warped": s["center_warped"], "diameter_px": s["diameter_px"]}
            for s in spots
        ],
    }
 
    # ---- 6. Build overlay & pop up ---------------------------------------
    if show:
        overlay = img.copy()
        cv2.drawContours(overlay, [box], 0, (255, 0, 0), 2)  # paper outline, blue
 
        for s in spots:
            pt = np.array([[s["center_warped"]]], dtype="float32")
            pt_o = cv2.perspectiveTransform(pt, M_inv)[0][0]
            radius = max(int(s["diameter_px"] / 2), 2)
            cv2.circle(overlay, (int(pt_o[0]), int(pt_o[1])), radius, (0, 255, 0), 1)
 
        cx, cy = int(smallest_center_orig[0]), int(smallest_center_orig[1])
        radius = max(int(smallest["diameter_px"] / 2), 2)
        cv2.circle(overlay, (cx, cy), radius + 6, (0, 0, 255), 3)
 
        arrow_start = (cx - 60, cy - 60)
        cv2.arrowedLine(overlay, arrow_start, (cx, cy), (0, 0, 255), 2, tipLength=0.3)
 
        label = f"smallest: {smallest['diameter_px']:.1f}px"
        if diameter_mm is not None:
            label += f" ({diameter_mm:.2f}mm)"
        cv2.putText(
            overlay, label, (arrow_start[0] - 10, arrow_start[1] - 10),
            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2, cv2.LINE_AA
        )
 
        cv2.imshow("Smallest burn mark", overlay)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
 
    return result
 
 
def _order_points(pts):
    """Order 4 corner points as bottom-left, top-left, top-right, bottom-right."""
    s = pts.sum(axis=1)
    diff = np.diff(pts, axis=1)
 
    top_left = pts[np.argmin(s)]
    bottom_right = pts[np.argmax(s)]
    top_right = pts[np.argmin(diff)]
    bottom_left = pts[np.argmax(diff)]
 
    return np.array([bottom_left, top_left, top_right, bottom_right], dtype="float32")


def main():
    # Setup all communication with hardware 
    camera_reg = setup()

    # # Calculate path for laser focal distance test
    robot_path, heights = calc_traj(camera_reg)

   # print("Smallest height", robot_path[72,2])

    # # # Run trajectory and measure diameters of laser's spot size at different heights
    # run_traj(camera_reg, robot_path, heights)

    # Measure diameters of burn marks!!!
    input(f"Press Enter to begin image analysis.")
    color_img = camera_reg.rgbd_cam.get_latest()['color']
    # Calculate the burn diameter from the image
    measure_burn_diameter(color_img)
    
    # TODO/FIXME: For measuring diameters of laser's aiming beam
    # # Determine the focal distance based on smallest diameter and its corresponding height
    # min_diameter_idx = np.nanargmin(diameters)
    # min_diamter = diameters[min_diameter_idx]
    # focal_distance = heights[min_diameter_idx]
    # print(f"Focal distance is: {focal_distance} m with beam waist diameter of: {min_diamter} pixels")
    # # Simple plot of findings
    # laser_heights = (heights + 0.08) * 1000
    # plt.scatter(laser_heights, diameters)
    # plt.xlabel("Laser Height (mm)")
    # plt.ylabel("Diameter (pixels)")
    # plt.title("Laser Height vs. Laser Spot Diameter")
    # plt.savefig("laser_height_vs_diameter.png")

if __name__ == "__main__":
    main()