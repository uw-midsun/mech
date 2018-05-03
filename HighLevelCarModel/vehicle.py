#!/usr/bin/env python

""" All vehcile related classes

Module contains all classes related to the vehicle as a whole, vehicle
parameters.

All units are as follows unless otherwise specified:
    Distance: meters (m)
    Mass: kilograms (kg)
    Time: seconds (s)
    Angles: degrees (deg)
    Temperature: degrees celcius (degC)

Midnight Sun Solar Car Team 2018
University of Waterloo
"""

__author__ = "Devon Copeland"
__copyright__ = "Copyright 2018, Midnight Sun Solar Car Team"

from datetime import datetime, timedelta
import math

class VehicleConstants:
    """Class to store all unchanging vehicle parameters"""
    
    def __init__(self):
        pass
    
    def load_vehicle(self, vehicle_dictionary):
        for key, value in vehicle_dictionary.iteritems():
            setattr(self, key, value)

class Coordiante:
    """Class to store waypoint on vehicle route"""
    
    TO_RADIANS = 0.0174533
    TO_DEGREES = 57.2957549575 
    
    def __init__(self, input_latitude, input_longitude):
        self.latitude = input_latitude
        self.longitude = input_longitude
        
    def get_distance_to(self, target_coordiante):
        """Funtion to compute the distance between two coordiantes in km.
        Implemented using the haversine formula: https://www.movable-type.co.uk/scripts/latlong.html"""
        
        R = 6371e3 # radius of the earth in metres
        latitude_1 = self.latitude * Coordiante.TO_RADIANS
        latitude_2 = target_coordiante.latitude * Coordiante.TO_RADIANS
        delta_latitude = (target_coordiante.latitude-self.latitude) * Coordiante.TO_RADIANS
        delta_longitude = (target_coordiante.longitude- self.longitude) * Coordiante.TO_RADIANS

        a = ( math.sin(delta_latitude/2) * math.sin(delta_latitude/2) +
              math.cos(latitude_1) * math.cos(latitude_2) *
              math.sin(delta_longitude/2) * math.sin(delta_longitude/2) )
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        
        return R * c;  
        
    def get_direction_to(self, target_coordiante):
        """Funtion to compute the distance between two coordiantes in km.
        Implemented using the forward azimuth formula: https://www.movable-type.co.uk/scripts/latlong.html"""
        
        latitude_1 = self.latitude * Coordiante.TO_RADIANS
        latitude_2 = target_coordiante.latitude * Coordiante.TO_RADIANS
        longitude_1 = self.longitude * Coordiante.TO_RADIANS
        longitude_2 = target_coordiante.longitude * Coordiante.TO_RADIANS
        
        y = math.sin(longitude_2 - longitude_1) * math.cos(latitude_2)
        x = ( math.cos(latitude_1) * math.sin(latitude_2) -
              math.sin(latitude_1) * math.cos(latitude_2) *
              math.cos(longitude_2 - longitude_1) )
        return math.atan2(y, x) * Coordiante.TO_DEGREES

class Waypoint:
    """Class to store waypoint on vehicle route"""
    
    def __init__(self):
        self.coordinate = Coordiante(NaN, NaN)     #next waypoint poition, deg  
        self.elevation = NaN             #current elevation above sea level, m
        self.target_velocity = NaN       #intended velocity at waypoint, m/s
        self.cloudiness = NaN            #factor of irradiance blocked by clouds, %

class Battery:
    """Class to handle all battery level funcitons"""
    
    def __init__(self, input_timestep):
        self.timestep = input_timestep
        self.soc = NaN                   #current battery pack state of charge, J
        self.net_power = [0, 0]          #fixed lenght FIFO with right side in, left side out
        
    def update(self, motor_power, rear_array_power, front_array_power,
               lv_losses, hv_losses):
        """Finds net power and integrates to update batter SoC. Note that in the
        context of this model, power generated is ALWAYS positive and power
        consumed is ALWAYS negative"""
        self.net_power.pop(0)
        self.net_power.append(motor_power + rear_array_power +
                              front_array_power + lv_losses + hv_losses)
        # Integerate using trapazoidal rule
        self.soc += ( self.net_power[0] +
                      self.net_power[1] * self.timestep / 2 )
 
class Driver:
    """Class for driver block"""
    
    def __init__(self, kP, kI, kD, timestep):
        self.kP = kP
        self.kI = kI
        self.kD = kD
        self.timestep = timestep
        self.error_integrator = 0
        self.velocity_error              #fixed length fifo to store last 2 velocity error terms
        self.commanded_throttle = NaN    #current commanded throttle, %
        self.brake_pedal_input = NaN     #current brake pedal force, N
        
    def update(self, curr_velocity, target_velocity):
        """Compuptes control signal and adjusts commended throttle and brake
        pedal input accordingly. Both are values from 0 to 1 represeting the
        amount the pedal is depressed. This will be mapped to torques in the
        brakes and motor classes"""
        self.velocity_error.pop(0)
        self.velocity_error.append(target_velocity-curr_velocity)
        self.error_integrator += ( self.velocity_error[0] +
                                   self.velocity_error[1] * self.timestep / 2 )
        # Compute control signal based on PID values
        control_signal = ( self.velocity_error[1] * self.kP +
                           self.error_integrator * self.kI +
                           (self.velocity_error[1] -self.velocity_error[0]) * self.kD )
        
        # Compute pedal commands based on control signal
        if control_signal > 1:
            self.commanded_throttle = 1
            self.brake_pedal_input = 0
        elif control_signal >= 0:
            self.commanded_throttle = control_signal
            self.brake_pedal_input = 0
        elif control_signal >= -1:
            self.commanded_throttle = 0
            self.brake_pedal_input = control_signal
        else:
            self.commanded_throttle = 0
            self.brake_pedal_input = 1
    
class Brakes:
    """Class for brakes block"""
    
    def __init__(self):
        self.mu_k_brakes = NaN
        self.mu_s_tire = NaN
        self.area_caliper_cyclinder = NaN
        self.max_line_pressue = NaN
        self.diam_brake_rotor = NaN
        self.diam_wheel = NaN
        self.num_brake_calipers = NaN
        self.torque = NaN          #current motor torque, Nm
        
    def brake_pedal_input_to_pressue(self, brake_pedal_input):
        """Function to map proportion of brake pedal stroke to line pressure"""
        return brake_pedal_input * self.max_line_pressure
    
    def update(self, brake_pedal_input, single_wheel_normal_force_front):
        """Computes torque from mechanical brakes"""
        line_pressure = self.brake_pedal_input_to_pressue(brake_pedal_input)
        brake_pad_force = line_pressure * self.area_caliper_cylinder
        
        # Torque = total friction * moment arm
        # Multiply by 2 for 2 brake pads per caliper. Divide by 2 for rotor radius
        total_brake_pad_torque = ( brake_pad_force * self.mu_k_brakes *
                                   self.num_brake_calipers * 2 * self.diam_brake_rotor / 2 )
        
        # Torque = front wheel normal force * tire static frition * moment arm
        # Multipy by 2 for 2 wheels. Divide by 2 for wheel radius
        
        max_tire_static_frition_torque = ( single_wheel_normal_force_front *
                                           self.mu_s_tire * 2 * self.diam_wheel / 2 )
        # Check if the wheels have locked up (if brake torque > tire static frction torque)
        if ( total_brake_pad_torque > max_tire_static_frition_torque):
            self.torque = max_tire_static_frition_torque
        else:
            self.torque = total_brake_pad_torque
        
class Motor:
    """Class for motor block"""
    
    def __init__(self):
        self.torque = NaN          #current motor torque, Nm
        self.power = NaN                 #current motor consumption, W
        
    def update(self, commanded_throttle):
        pass
    
class CarDynamics:
    """Class for car dynamics block"""
    
    def __init__(self):
        # lower indices are most recent
        self.velocity = [0,0,0]              # velocity list, m/s
        self.acceleration = [0,0,0]          # acceleration list, m/s^2
        
        def update(self, motor_torque, brake_torque):
            pass
    
class Array:
    """Class for array block"""
    
    def __init__(self):
        self.power = NaN                 #current front array power produced, W
        
        def update(self, irradiance):
            pass

class Sunlight:
    """Class for sunlight block"""
    
    def __init__(self):
        self.irradiance = NaN            #curent irradiance on car, W/m^2
        
    def update(self, position, elevation, cloudienss, date_time):
        pass
    
class HighVoltageLosses:
    """Class for high voltage losses block"""
    
    def __init__(self): 
        self.power = NaN                 #current HV system consumption, W
        
        def update(self):
            pass
    
class LowVoltageLosses:
    """Class for low voltage losses block"""
    
    def __init__(self):
        self.power = NaN                 #current LV system consumption, W
        
        def update(self):
            pass

class VehicleLog:
    """Class logging data after each iteration"""

    def __init__(self):
        self.date_time = []
        self.position = []
        self.elevation = []
        self.heading = []
        self.gradient = []
        self.total_distance = []
        self.battery_soc = []
        self.commaned_throttle = []
        self.brake_pedal_force = []
        self.mech_brake_torque = []
        self.motor_torque = []
        self.motor_power = []
        self.velocity = []
        self.acceleration = []
        self.irradiance = []
        self.front_array_power = []
        self.rear_array_power = []
        self.hv_losses = []
        self.lv_losses = []
        
class Vehicle:
    """Class to handle all highest level vehicle functions"""
    
    TIMESTEP = 1                         #timestep for iterataive calcs, s
    WAYPOINT_TOL = 50                    #tolerance for reaching waypoint, m
    
    def __init__(self):
        # Vehicle level state variables
        self.date_time = datetime()
        self.position = Coordiante(NaN, NaN)  #current poition, deg
        self.elevation = NaN                  #current elevation above sea level, m
        self.heading = NaN                    #direction of travel, deg
        self.gradient = NaN                   #slope of current gradient, rise/run
        self.total_distance = NaN             #totat distance traveled, m
        
        # Classes that represent blocks in the system model. These blocks
        # contain and maipulate other state variables
        self.prev_waypoint = Waypoint()        # waypoint most recently passed
        self.next_waypoint = Waypoint()        # next waypoint to drive to
        self.driver = Driver(NaN,NaN,NaN)           
        self.motor = Motor()
        self.mech_brakes = Brakes()
        self.car_dynamics = CarDynamics(Vehicle.TIMESTEP)
        self.rear_array = Array()
        self.front_array = Array()
        self.sunlight = Sunlight()
        self.battery = Battery(TIMESTEP)
        self.lv_losses = LowVoltageLosses()
        self.hv_losses = HighVoltageLosses()
        
        # Logfile
        self.log = VehicleLog()
        
    def log_datapoint(self):
        self.log.date.append(self.date_time)
        self.log.position.append(self.position)
        self.log.elevation.append(self.elevation)
        self.log.heading.append(self.heading)
        self.log.gradient.append(self.gradient)
        self.log.total_distance.append(self.total_distance)
        self.log.battery_soc.append(self.battery.soc)
        self.log.commaned_throttle.append(self.driver.commaned_throttle)
        self.log.brake_pedal_force.append(self.driver.brake_pedal_input)
        self.log.mech_brake_torque.append(self.mech_brakes.torque)
        self.log.motor_torque.append(self.motor.torque)
        self.log.motor_power.append(self.motor.power)
        self.log.velocity.append(self.car_dynamics.velocity)
        self.log.acceleration.append(self.car_dynamics.acceleration)
        self.log.irradiance.append(self.sunlight.irradiance)
        self.log.front_array_power.append(self.front_array.power)
        self.log.rear_array_power.append(self.rear_array.power)
        self.log.hv_losses.append(self.hv_losses.power)
        self.log.lv_losses.append(self.lv_losses.power)
        
    def drive_to_next_waypoint(self):
        """Function to iterate at period of timestep untill the vehicle has
        reached the target waypoint within the given tolerance"""
        
        # Solve for original distance, heading and graident
        dist_to_waypoint = self.position.get_distance_to(self.next_waypoint)
        self.heading = self.position.get_direction_to(self.next_waypoint)
        self.gradient = (self.next_waypoint.elevation - self.elevation) / dist_to_waypoint
        
        # Iterate untill below toelerance or distance to waypoint begins to increase
        while(dist_to_waypoint > Vehicle.WAYPOINT_TOL):
            self.update_vehicle_state()
            # Update distance traveled
            delta_distance = abs( self.car_dynamics.velocity[0] +
                                  self.car_dynamics.velocity[1] ) * TIMESTEP / 2
            self.total_distance += delta_distance
            # Update elevation 
            self.elevation += self.gradient * delta_distance            
            # Update time
            self.date_time += timedelta(millisecond=(Vehicle.TIMESTEP*1000))
            dist_to_waypoint -= delta_distance
            log_datapoint()
            
        # Update position to be target waypoint once reached
        self.position = self.next_waypoint.coordinate

    def update_vehicle_state(self):
        """Function to..."""
        self.sunlight.update(self.position, self.elevation,
                             self.cloudiness, self.date_time)
        self.front_array.update(self.sunlight.irradiance)
        self.rear_array.update(self.sunlight.irradiance)
        self.driver.update(self.car_dynamics.velocity,
                           self.prev_waypoint.target_speed)
        self.motor.update(self.driver.commanded_throttle)
        self.mech_brakes.update(self.driver.brake_pedal_input)
        self.car_dynamics.update(self.motor.torque, self.mech_brakes.torque)
        self.lv_losses.update()
        self.hv_losses.update()
        self.battery.update(self.motor.power, self.rear_array.power,
                            self.front_array.power,
                            self.lv_losses.power,
                            self.hv_losses.power)
        