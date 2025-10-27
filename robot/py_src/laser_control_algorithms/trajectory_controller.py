import os, sys

import numpy as np
from laser_control_algorithms.laser_controller import LaserController
from laser_control_algorithms.controller_message import ControllerMessage

from laser_control_algorithms.trajectory_helpers.quintic_trajectory import QuinticTrajectory
from laser_control_algorithms.trajectory_helpers.path_generation import Path_Gen

class TrajectoryController(LaserController):
    
    def __init__(self,controller_msg : ControllerMessage):
        # Abstract class attributes
        self._laser_pos = np.array([0,0,0])
        self._laser_vel = np.array([0,0,0])
        self._laser_on = False
        self._irradiance = 0


        self.laser_info = controller_msg.laser_info
        self.gains = controller_msg.gains
        
        if self.positions.shape[0]-1 != len(self.durations):
            raise Exception("Durations and Postions dims do not match")
        self.positions, self.durations = self.generatePath() # Generates new points depending on pattern
        self.generateTrajectories(self.positions, self.durations, self.max_velocity, self.max_acceleration)

        self.t = 0
        self.time_step = controller_msg.time_step
        self.pulse_idx = 0
        self.debug = controller_msg.debug
    
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
            
        elif(self.pattern == "Line"):
            points = self.positions.copy()
            durations = self.durations.copy()
            return points, durations
        else:
            print("Pattern: ", self.pattern)
            raise ValueError("Trajectory Controller can not handle pattern above")
        
        distances = np.linalg.norm(points[1:] - points[:-1], axis=1)
        durations = (distances / np.sum(distances)) * self.durations[0]
        cumulative_durations = np.cumsum(durations)
        points = np.hstack((points, np.full((points.shape[0], 1), self.positions[0, -1])))
        
        return points, cumulative_durations
        
    def generateTrajectories(self, position, durations, maxVelocity, maxAcceleration):
        self.n_points, self.n_dims = position.shape
        
        # Check this. Almost certain that this does not mean max velocity and acceleration
        # Pretty sure this is just starting and final velocity and acceleration
        
        self.MAX_VELOCITY = np.full(self.n_dims, maxVelocity) # shape:(x) !!!!!! FIND UNITS [m/s]
        self.MAX_ACCELERATION = np.full(self.n_dims, maxAcceleration) #!!!!!! FIND UNITS [m/s^2]

        self.trajectories = np.empty((self.n_points - 1, self.n_dims), dtype=object) # shape:(n,x)
        
        for i in range(self.n_points - 1):
            t0 = 0 if i == 0 else durations[i-1] # Initial time 
            tf = durations[i] # Final time
            
            p0 = position[i] # Initial Position
            pf = position[i+1] # Final Position
            
            v0 = np.zeros(self.n_dims) if i == 0 else self.MAX_VELOCITY # Initial Velocity
            vf = np.zeros(self.n_dims) if i == self.n_points - 2 else self.MAX_VELOCITY  # Final Velocity
            
            a0 = np.zeros(self.n_dims) # Initial Acceleration
            af = np.zeros(self.n_dims) # Final Accerlation
            
            for dim in range(self.n_dims):
                self.trajectories[i, dim] = QuinticTrajectory(t0 = t0, tf = tf,
                                                              p0 = p0[dim], pf = pf[dim],
                                                              v0 = v0[dim], vf = vf[dim],
                                                              a0 = a0[dim], af = af[dim])

    def update(self,controller_msg : ControllerMessage):
        self.t += self.time_step
        # We default to the laser being off at the given height
        self._irradiance = 0
        self._laser_on = False        

        # Calculate Trajectory 
        idx = np.searchsorted(self.durations, self.t, side='right')
        pathIdx = idx if idx < self.n_points-2 else self.n_points-2
        self._laser_pos = np.zeros(self.n_dims) # laser position in [cm] shape:(x)
        
        for dim in range(self.n_dims):
            self._laser_pos[dim] = self.trajectories[pathIdx, dim].position(self.t)
        
        self._laser_on = True        
            
        if self._laser_on: # if the laser is on, calculate the beam width and expected peak irradiance
            power = controller_msg.power
            width = self.laser_info['w0']*np.sqrt(1 + ((self._laser_pos[2] * self.laser_info['wavelength'])/(np.pi*self.laser_info['w0']**2))**2) 
            self._irradiance = 2*power/(np.pi*(width**2))
        if self.debug:
            print("Spot Temperature: ", np.amax(controller_msg.current_temp))
            print("Irradiance: ", self._irradiance)
            print("Laser Position: ", self._laser_pos)
            print("Current Path: ", self.positions[pathIdx+1])
            print("Time: ", self.t)
            print("")

        return (self._laser_pos,self._laser_on, self._irradiance)
    
    def _set_gains(self):
        self.positions = np.array(self._gains["Positions"]) # laser position in [cm] shape:(n,x)
        self.durations = np.array(self._gains["Durations"]) # shape: (x) 
        self.pattern: str = self._gains["Pattern"]
        self.num_passes: int = self._gains["Passes"]
        self.max_acceleration = self._gains["MaxAcceleration"]
        self.max_velocity = self._gains["MaxVelocity"]