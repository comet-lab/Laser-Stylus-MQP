class ControllerMessage:
    # Stores all parameters and necessary information that any concrete implementation of hte controller could need

    def __init__(self,time_step,starting_temp,reference_temp,gains,laser_info,tissue_info,\
            debug=False,params=dict()):
        self.time_step = time_step
        self.starting_temp = starting_temp
        self.gains = gains
        self.laser_info = laser_info
        self.tissue_info = tissue_info
        self.params = params
        self.debug = debug
        self.set_current_temp(starting_temp)
        self.set_reference_temp(reference_temp)
        self.set_reference_temp_dt(0)
        self.set_tissue_location([0,0])
        self.set_power(0)


    def update(self,current_temp,tissue_loc,reference_temp,reference_temp_dt,power=1):
        self.set_current_temp(current_temp)
        self.set_tissue_location(tissue_loc)
        self.set_reference_temp(reference_temp)
        self.set_reference_temp_dt(reference_temp_dt)
        self.set_power(power)

    def set_power(self, power):
        self.power = power

    def set_current_temp(self, current_temp):
        self.current_temp = current_temp

    def set_tissue_location(self,tissue_loc):
        self.tissue_loc = tissue_loc
    
    def set_reference_temp(self, reference_temp):
        self.reference_temp = reference_temp

    def set_reference_temp_dt(self, reference_temp_dt):
        self.reference_temp_dt = reference_temp_dt
