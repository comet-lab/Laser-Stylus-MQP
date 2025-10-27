from abc import ABC, abstractmethod
from laser_control_algorithms.controller_message import ControllerMessage

class LaserController(ABC):

    # ABSTRACT PROPERTIES
    @property
    @abstractmethod
    def name(self):
        """Each implmentation must define its own name"""
        pass

    # ABSTRACT METHODS
    @abstractmethod
    def __init__(self, controller_msg : ControllerMessage):
        """Initialize the controller with a message"""
        pass

    @abstractmethod
    def update(self, controller_msg : ControllerMessage):
        """Run the update method on the controller with a controller message"""
        pass

    @abstractmethod
    def _set_gains(self):
        """Set the gains of the controller, used for gains setter"""

    # CONCRETE PROPERTIES

    @property
    def type(self):
        allGainsString = ""
        gains = self.gains
        for key in gains.keys():
            # for each gain type, add the gain name and the gain value to the filename string
            # make sure to remove spaces if there is an array 
            if isinstance(gains[key],list): # list gets converted to string as '[1, 2]'
                gainString = str.replace(str(gains[key])," ","")
            else: # numpy array gets converted to string as '[1 2]'
                gainString = str.replace(str(gains[key])," ",",")
            allGainsString += str(key) + "-" + gainString + "_"
        allGainsString = allGainsString[:-1] # remove last underscore
        return allGainsString

    @property
    def gains(self):
        return self._gains

    @gains.setter
    def gains(self,value):
        self._gains = value
        self._set_gains()

    @property
    def irradiance(self):
        return self._irradiance
    
    @property
    def laser_on(self):
        return self._laser_on
    
    @property
    def laser_pos(self):
        return self._laser_pos
    
    @property
    def laser_vel(self):
        return self._laser_vel
