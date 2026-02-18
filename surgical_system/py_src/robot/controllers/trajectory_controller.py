import numpy as np
import matplotlib.pyplot as plt
# from laser_controller import LaserController

from .trajectory_helpers.quintic_trajectory import QuinticTrajectory
from .trajectory_helpers.path_generation import Path_Gen
import math

class TrajectoryController():
    
    def __init__(self, path_info, debug = True):
        # Abstract class attributes
        self.init_path(path_info)
        self.laser_pos = np.array([0,0,0])
        self.laser_pos_vel = np.array([0,0,0])
        self._laser_on = False
        self._irradiance = 0
        self.debug = debug
        
        # self.laser_info = controller_msg.laser_info
        
        if self.positions.shape[0]-1 != len(self.durations) and not self.pattern == "Circle" and not self.pattern == "Custom":
            raise Exception("Durations and Postions dims do not match")
        self.positions, self.durations = self.generatePath() # Generates new points depending on pattern
        self.generateTrajectories(self.positions, self.durations, self.max_velocity, self.max_acceleration)

        self.t = 0
        # self.time_step = controller_msg.time_step
    
    @property
    def name(self):
        return "Trajectory-Controller"
    
    @property
    def type(self):
        typeString = self.pattern
        if (self.pattern == "Raster"):
            typeString = typeString + "-" + str(self.num_passes)
        return typeString
        
    def generatePath(self):
        if(self.pattern == "Raster"):
            if self.positions.size != 6:
                raise ValueError("Raster pattern can only accept 2 points from Positions.")
            if self.durations.size != 1:
                raise ValueError("Raster pattern can only accept 1 time duration.")
            points = Path_Gen.generateRaster(self.positions[0, :2],  self.positions[1, :2], self.num_passes)

        elif(self.pattern == "Polygon"): # edit this to complete polygon
            points = self.positions.copy()
            durations = self.durations.copy()
        
        elif(self.pattern == "Custom"): # Custom Path
            points = self.positions.copy()
            
        elif(self.pattern == "Line"):
            points = self.positions.copy()
            durations = np.cumsum(self.durations)
            return points, durations
        elif(self.pattern == "Circle"):
            points = Path_Gen.generate_circle(self.positions[0, :2], self.radius)
            
        else:
            print("Pattern: ", self.pattern)
            raise ValueError("Trajectory Controller can not handle pattern above")
        
        diffs = points[1:] - points[:-1]
        dist = np.linalg.norm(diffs, axis=1)
        
        distances = np.linalg.norm(points[1:] - points[:-1], axis=1)
        eps = 1e-6  # if the distances are too small, remove
        mask = distances > eps

        if not np.any(mask):
            raise ValueError("Path has no non-trivial segments (all points are identical).")

        # rebuild points using only non-zero-length segments
        points = np.vstack((points[0], points[1:][mask]))
        distances = dist[mask]


        self.total_distance = np.sum(distances)
        if self.durations[0] == -1:
            self.durations[0] = self.total_distance/float(self.max_velocity)
            print(self.durations[0])
            
        durations = (distances / np.sum(distances)) * self.durations[0]
        cumulative_durations = np.cumsum(durations)
        if points.shape[-1] < 3: # Points are not 3D
            points = np.hstack((points, np.full((points.shape[0], 1), self.positions[0, -1])))
        
        return points, cumulative_durations
        
    def generateTrajectories(self, position, durations, maxVelocity, maxAcceleration):
        self.n_points, self.n_dims = position.shape
        
        # Check this. Almost certain that this does not mean max velocity and acceleration
        # Pretty sure this is just starting and final velocity and acceleration
        
        self.MAX_VELOCITY = np.full(self.n_dims, maxVelocity) # shape:(x) !!!!!! FIND UNITS [m/s]
        self.MAX_ACCELERATION = np.full(self.n_dims, maxAcceleration) #!!!!!! FIND UNITS [m/s^2]

        durations = np.insert(durations, 0, 0) 
        v_way = np.zeros_like(position)          # (N, D)

        # endpoints
        v_way[0]  = 0.0
        v_way[-1] = 0.0
        for i in range(1, self.n_points-1):
            dt = durations[i] - durations[i-1]
            if dt <= 0:
                raise ValueError("durations must be strictly increasing")
            v_way[i] = (position[i] - position[i-1]) / dt
        
        a_way = np.zeros_like(position)
        self.trajectories = np.empty((self.n_points - 1, self.n_dims), dtype=object)
        for i in range(self.n_points - 1):
            t0 = durations[i]
            tf = durations[i + 1]

            p0 = position[i]
            pf = position[i + 1]

            v0 = v_way[i]
            vf = v_way[i + 1]

            a0 = a_way[i]
            af = a_way[i + 1]


            for d in range(self.n_dims):
                self.trajectories[i, d] = QuinticTrajectory(
                    t0=t0, tf=tf,
                    p0=p0[d], pf=pf[d],
                    v0=v0[d], vf=vf[d],
                    a0=a0[d], af=af[d]
                )
        
        time_step = 0.05
        data_num = int(math.ceil(durations[-1]/time_step)) 
        actual_vel_list = np.empty((data_num, 3))
        times = np.linspace(0, durations[-1], data_num)
        for i in range(data_num):
            movement = self.update(times[i])
            target_vel = (movement['velocity'])
            actual_vel_list[i] = target_vel
            
        plt.figure(figsize=(6,4))
        plt.plot(times, actual_vel_list[:, 0], label="actual x")
        plt.plot(times, actual_vel_list[:, 1], label="actual y")
        plt.xlabel("Time [s]")
        plt.ylabel("velcity [m]")
        plt.title("Time vs Position (Guess)")
        plt.grid(True)
        plt.legend()
        plt.tight_layout()
        plt.savefig("trajectory.png")
        # plt.show()
        # plt.close(3)

    def update(self, time):
        self.t = time
        # We default to the laser being off at the given height
        # self._irradiance = 0
        # self._laser_on = False        

        # Calculate Trajectory 
        idx = np.searchsorted(self.durations, self.t, side='right')
        pathIdx = idx if idx < self.n_points-2 else self.n_points-2
        self.laser_pos = np.zeros(self.n_dims) # laser position in [cm] shape:(x)
        self.laser_pos_vel = np.zeros(self.n_dims)
        
        for dim in range(self.n_dims):
            self.laser_pos[dim] = self.trajectories[pathIdx, dim].position(self.t)
            self.laser_pos_vel[dim] = self.trajectories[pathIdx, dim].velocity(self.t)
            
        traj = {'position': self.laser_pos,
                'velocity': self.laser_pos_vel}
                    
        # if self._laser_on: # if the laser is on, calculate the beam width and expected peak irradiance
            # power = controller_msg.power
            # width = self.laser_info['w0']*np.sqrt(1 + ((self._laser_pos[2] * self.laser_info['wavelength'])/(np.pi*self.laser_info['w0']**2))**2) 
            # self._irradiance = 2*power/(np.pi*(width**2))
        if self.debug:
            pass
            # print("Spot Temperature: ", np.amax(controller_msg.current_temp))
            # print("Irradiance: ", self._irradiance)
            # print("Laser Position [m]: ", self.laser_pos)
            # print("Laser Velocity [m/s]: ", self.laser_pos_vel)
            # print("Current Path: ", self.positions[pathIdx+1])
            # print("Time: ", self.t)
            # print("")

        return traj
    
    def init_path(self, path_info):
        self.positions = np.array(path_info["Positions"], dtype=float) # laser position in [cm] shape:(n,x)
        self.start_pos = self.positions[0]
        self.durations = np.array(path_info["Durations"], dtype=float) # shape: (x)
        self.pattern: str = path_info["Pattern"]
        self.num_passes: int = path_info["Passes"]
        self.max_acceleration = path_info["MaxAcceleration"]
        self.max_velocity = path_info["MaxVelocity"]
        self.radius = path_info["Radius"]