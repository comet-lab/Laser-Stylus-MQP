import numpy as np
from laser_control_algorithms.laser_controller import LaserController
from laser_control_algorithms.controller_message import ControllerMessage

class NoController(LaserController):

    def __init__(self,controller_msg : ControllerMessage):
        self.laser_info = controller_msg.laser_info

        self._laser_pos = np.array([0,0,0])
        self._laser_vel = np.array([0,0,0])
        self._laser_on = False
        self._irradiance = 0
        # height and pulseDuration can be lists or single values. If they are lists, we assume we are pulsing the laser, and each 
        # index alternates between the laser being on and off. If they are single values, we assume we just keep the laser at the 
        # given height for the onDuration.
        self.gains = controller_msg.gains # calls setter function
        # Compare the lists to make sure they are the same lengths
        if len(self.height_targets) != len(self.pulse_duration):
            raise Exception("Height and Duration lists must have the same number of elements")
        self.t = 0
        self.time_step = controller_msg.time_step
        self.pulse_idx = 0
        self.debug = controller_msg.debug
        
    @property
    def name(self):
        return "No-Controller"
    
    @property
    def type(self):
        fullGainString = "Height-" + str.replace(str(self.height_targets)," ",",") + "_"\
              + "Duration-" + str.replace(str(self.pulse_duration)," ",",")
        return fullGainString

    def update(self, controller_msg : ControllerMessage):
        self.t += self.time_step
        # We default to the laser being off at the given height
        self._irradiance = 0
        self._laser_pos[2] = self.height_targets[self.pulse_idx]
        self._laser_on = False
         # if the current time is below the current duration 
        if (self.t < self.pulse_duration[self.pulse_idx]):
            if (self.pulse_idx%2 == 0): # AND the pulseIdx is even then we are firing the laser
                self._laser_on = True
            # If the pulseIDX is odd, then we simply keep the laser off and don't have to do anything
        
        elif self.pulse_idx < (len(self.height_targets)-1): 
            # if we are over the duration threshold, then we need to increment our pulseIdx and update other heights and durations
            # The extra condition makes sure we don't over index the array and leaves us in default off
            # If any duration passed in is 0, this will not be handled properly
            self.pulse_idx += 1
            self._laser_pos[2] = self.height_targets[self.pulse_idx]
            self._laser_on = (self.pulse_idx%2 == 0)
            
        if self._laser_on: # if the laser is on, calculate the beam width and expected peak irradiance
            power = controller_msg.power
            width = self.laser_info['w0']*np.sqrt(1 + ((self.height_targets[self.pulse_idx] * self.laser_info['wavelength'])/(np.pi*self.laser_info['w0']**2))**2) 
            self._irradiance = 2*power/(np.pi*(width**2))
        if self.debug:
            print("Controller: Spot Temperature = ", np.amax(controller_msg.current_temp))
            print("Controller: Irradiance = ", self._irradiance)
            print("Controller: Laser Height = ", self.height_targets[self.pulse_idx])
            print("")
        return (self._laser_pos, self._laser_on, self._irradiance)

    def _set_gains(self):
        self.height_targets = np.array(self._gains["Height"]) # laser height in [cm]
        self.pulse_duration = np.array(self._gains["Duration"])  # how long to keep the laser fiber on [seconds]