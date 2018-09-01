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
import numpy
import matplotlib.pyplot as plt
import csv

"""Trig functions implemented for degrees"""
def cosd(deg):
    return math.cos(math.radians(deg))

def sind(deg):
    return math.sin(math.radians(deg))

def tand(deg):
    return math.tan(math.radians(deg))

def acosd(ratio):
    return math.degrees(math.acos(ratio))

def asind(ratio):
    return math.degrees(math.asin(ratio))

def atand(ratio):
    return math.degrees(math.atan(ratio))

class VehicleConstants:
    """Class to store all unchanging vehicle parameters"""
    
    def __init__(self):
        pass
    
    def load_vehicle(self, vehicle_dictionary):
        print
        print "- - - - Vehicle Parameters: - - - - "
        for key, value in vehicle_dictionary.iteritems():
            setattr(self, key, value)
            print key + ": " + str(value)
        print

class Coordinate:
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
        latitude_1 = self.latitude * Coordinate.TO_RADIANS
        latitude_2 = target_coordiante.latitude * Coordinate.TO_RADIANS
        delta_latitude = (target_coordiante.latitude-self.latitude) * Coordinate.TO_RADIANS
        delta_longitude = (target_coordiante.longitude- self.longitude) * Coordinate.TO_RADIANS

        a = ( math.sin(delta_latitude/2.0) * math.sin(delta_latitude/2.0) +
              math.cos(latitude_1) * math.cos(latitude_2) *
              math.sin(delta_longitude/2.0) * math.sin(delta_longitude/2.0) )
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        
        return R * c;  
        
    def get_direction_to(self, target_coordiante):
        """Funtion to compute the direction to point to target coordinate.
        Implemented using the forward azimuth formula: https://www.movable-type.co.uk/scripts/latlong.html"""
        
        latitude_1 = self.latitude * Coordinate.TO_RADIANS
        latitude_2 = target_coordiante.latitude * Coordinate.TO_RADIANS
        longitude_1 = self.longitude * Coordinate.TO_RADIANS
        longitude_2 = target_coordiante.longitude * Coordinate.TO_RADIANS
        
        y = math.sin(longitude_2 - longitude_1) * math.cos(latitude_2)
        x = ( math.cos(latitude_1) * math.sin(latitude_2) -
              math.sin(latitude_1) * math.cos(latitude_2) *
              math.cos(longitude_2 - longitude_1) )
        return math.atan2(y, x) * Coordinate.TO_DEGREES

class Waypoint:
    """Class to store waypoint on vehicle route"""
    
    def __init__(self, inputs):
        # Order of inputs: lat, longi, elev, targ_vel, cloud, temp, reflect, humid
        self.coordinate = Coordinate(inputs[0], inputs[1])  #next waypoint poition, deg  
        self.elevation = inputs[2]                          #current elevation above sea level, m
        self.target_speed = inputs[3]                    #intended velocity at waypoint, m/s
        self.cloudiness = inputs[4]                         #factor of irradiance blocked by clouds, %
        self.temperature = inputs[5]                        #current temperature, degC
        self.reflectance = inputs[6]                        #current ground refectance, 
        self.humidity = inputs[7]                           #relative humidity, %

class Battery:
    """Class to handle all battery level funcitons"""
    
    def __init__(self, input_timestep, initial_soc):
        self.timestep = input_timestep
        self.soc = initial_soc           #current battery pack state of charge, J
        self.net_power = [0, 0]          #fixed lenght FIFO with right side in, left side out
        
    def update(self, motor_power, rear_array_power, front_array_power,
               lv_losses, hv_losses):
        """Finds net power and integrates to update battery SoC. Note that in the
        context of this model, power generated is ALWAYS positive and power
        consumed is ALWAYS negative"""
        self.net_power.pop(0)
        self.net_power.append(motor_power + rear_array_power +
                              front_array_power + lv_losses + hv_losses)
        # Integerate using trapazoidal rule
        self.soc += ( (self.net_power[0] +
                      self.net_power[1]) * self.timestep / 2.0 )
 
class Driver:
    """Class for driver block"""
    
    def __init__(self, kP, kI, kD, timestep):
        self.kP = kP
        self.kI = kI
        self.kD = kD
        self.timestep = timestep
        self.error_integrator = 0
        self.velocity_error = [0, 0]     #fixed length fifo to store last 2 velocity error terms
        self.commanded_throttle = 0      #current commanded throttle, %
        self.brake_pedal_input = 0       #current brake pedal force, N
        self.control_signal = 0
        
    def update(self, curr_velocity, target_speed):
        """Compuptes control signal and adjusts commended throttle and brake
        pedal input accordingly. Both are values from 0 to 1 represeting the
        amount the pedal is depressed. This will be mapped to torques in the
        brakes and motor classes"""
        self.velocity_error.pop(0)
        self.velocity_error.append(target_speed-curr_velocity)
        self.error_integrator += ( (self.velocity_error[0] +
                                   self.velocity_error[1]) * self.timestep / 2.0 )

        # Compute control signal based on PID values
        self.control_signal = ( self.velocity_error[1] * self.kP +
                                self.error_integrator * self.kI +
                               (self.velocity_error[1] - self.velocity_error[0]) * self.kD )
        
        # Compute pedal commands based on control signal
        if self.control_signal > 1:
            self.commanded_throttle = 1
            self.brake_pedal_input = 0
        elif self.control_signal >= 0:
            self.commanded_throttle = self.control_signal
            self.brake_pedal_input = 0
        elif self.control_signal >= -1:
            self.commanded_throttle = 0
            self.brake_pedal_input = abs(self.control_signal)
        else:
            self.commanded_throttle = 0
            self.brake_pedal_input = 1
    
class Brakes:
    """Class for brakes block"""
    
    def __init__(self, mu_k_brakes, mu_s_tire, mu_k_tire, area_caliper_cyl, max_line_pressure,
                 diam_brake_rotor, diam_wheel, num_brake_calipers_per_wheel, num_brake_wheels):
        self.mu_k_brakes = mu_k_brakes
        self.mu_s_tire = mu_s_tire
        self.mu_k_tire = mu_k_tire
        self.area_caliper_cylinder = area_caliper_cyl
        self.max_line_pressure = max_line_pressure
        self.diam_brake_rotor = diam_brake_rotor
        self.diam_wheel = diam_wheel
        self.num_brake_calipers_per_wheel = num_brake_calipers_per_wheel
        self.num_brake_wheels = num_brake_wheels
        self.torque = 0          #current motor torque, Nm
        
    def brake_pedal_input_to_pressue(self, brake_pedal_input):
        """Function to map proportion of brake pedal stroke to line pressure.
        [IMPROVEMENT] Function has been left trivial for future improvments
        (perhaps brake pedal input maps to stroke instead of % of max pressure)"""
        if brake_pedal_input < 0.3:
            brake_pedal_input = 0
        return brake_pedal_input * self.max_line_pressure
    
    def update(self, brake_pedal_input, single_wheel_normal_force_front):
        """Computes torque from mechanical brakes"""
        line_pressure = self.brake_pedal_input_to_pressue(brake_pedal_input)
        brake_pad_force = line_pressure * self.area_caliper_cylinder
        
        # Torque = total friction * moment arm
        # Multiply by 2 for 2 wheels. Divide by 2 for rotor radius
        total_brake_pad_torque = ( brake_pad_force * self.mu_k_brakes *
                                   self.num_brake_calipers_per_wheel *
                                   self.num_brake_wheels  * self.diam_brake_rotor / 2.0 )
        
        # Max torque = front wheel normal force * tire static frition * moment arm
        # Multipy by 2 for 2 wheels. Divide by 2 for wheel radius
        
        max_tire_static_frition_torque = ( single_wheel_normal_force_front *
                                           self.mu_s_tire * self.num_brake_wheels *
                                           self.diam_wheel / 2.0 )
        
        max_tire_dynamic_frition_torque = ( single_wheel_normal_force_front *
                                           self.mu_k_tire * self.num_brake_wheels *
                                           self.diam_wheel / 2.0 )
        
        # Check if the wheels have locked up (if brake torque > tire static frction torque)
        if ( total_brake_pad_torque > max_tire_static_frition_torque):
            self.torque = max_tire_dynamic_frition_torque
        else:
            self.torque = total_brake_pad_torque
        
class Motor:
    """Class for motor block"""
    
    TO_RAD_PER_SECOND = 0.1047197177
    
    def __init__(self, mu_s_tire, mu_k_tire, diam_wheel, stall_torque,
                 no_load_speed, regen_gain, num_motors):
        self.torque = 0          #current motor torque, Nm
        self.power = 0           #current motor consumption, W
        self.mu_s_tire = mu_s_tire
        self.mu_k_tire = mu_k_tire
        self.diam_wheel = diam_wheel
        self.stall_torque = stall_torque
        self.no_load_speed = no_load_speed
        self.regen_gain = regen_gain #regen gain must be negative
        self.num_motors = num_motors
        
    def get_torque_from_throttle(self, commanded_throttle, curr_rpm):
        """Function to map throttle to motor torque. Assume linear model
           until torque speed curve can be input"""
        torque_vs_speed_slope = - (float(self.stall_torque) / float(self.no_load_speed))
        speed_adjusted_max_torque = self.stall_torque + torque_vs_speed_slope * curr_rpm
        return speed_adjusted_max_torque * commanded_throttle * self.num_motors

    def get_drive_efficiency(self, curr_rpm):
        """Function to get motor drive efficiency from torque and speed"""
        return 0.8 #[IMPROVEMENT] Set as constant for now
    
    def get_regen_efficiency(self, curr_rpm):
        """Function to get motor regen braking efficiency from torque and speed"""
        return 0.8 #[IMPROVEMENT] Set as constant for now
        
    def update(self, commanded_throttle, curr_velocity, single_wheel_normal_force_rear, target_speed):
        """Function to update motor toque"""
        curr_rpm = curr_velocity * 60 / (math.pi * self.diam_wheel)
        if commanded_throttle == 0: # Check for regen braking, use linear model
            error = target_speed - curr_velocity
            self.torque = self.regen_gain * error * self.num_motors
            self.power = (self.torque * curr_rpm * Motor.TO_RAD_PER_SECOND *
                          self.get_regen_efficiency(curr_rpm))
        else: 
            self.torque = self.get_torque_from_throttle(commanded_throttle, curr_rpm)
            self.power = (self.torque * curr_rpm * Motor.TO_RAD_PER_SECOND /
                          self.get_drive_efficiency(curr_rpm))
        #Flip the sign of power to correspond with sign convension
        # [IMPROVEMENT] this is a hack fix for now need to change sign or torque for entire model 
        self.power *= -1
        #[IMPROVEMENT] Could add logic to check for tire slipping here
    
class CarDynamics:
    """Class for car dynamics block"""
    
    TO_RADIANS = 0.0174533
    A_GRAVITY = 9.81
    
    def __init__(self, mass, c_viscous, c_coulombic, c_drag, wheel_diam, wheel_j,
                 motor_rotor_j, timestep, frontal_area, wheelbase, cog_height):
        # higher indices are most recent
        self.velocity = [0,0]              # velocity list, m/s
        self.acceleration = [0,0]          # acceleration list, m/s^2
        self.mass = mass
        self.c_viscous = c_viscous
        self.c_coulombic = c_coulombic
        self.c_drag = c_drag
        self.wheel_diam = wheel_diam
        self.wheel_rad = wheel_diam/2.0
        self.wheel_j = wheel_j
        self.motor_rotor_j = motor_rotor_j
        self.frontal_area = frontal_area
        self.wheelbase = wheelbase
        self.cog_height = cog_height
        self.curr_road_gradient = 0
        self.curr_motor_torque = 0
        self.curr_brake_torque = 0
        self.curr_air_density = 1.2
        self.timestep = timestep
        self.normal_force_front = (self.A_GRAVITY*mass/4.0)
        
    def get_air_density(self, elevation, temperature, humidity):
        # Funtion to determine air density based on temperature, pressure
        # and humidity. Uses partial pressures of dry air and water vapour
        # https://wahiduddin.net/calc/density_altitude.htm
        # Air pressure calculated based on elevation (assume changes due to weather are negligible)
        # https://en.wikipedia.org/wiki/Atmospheric_pressure
        air_pressure = 101325 * math.pow((1 - elevation * 0.00002255769564), 5.2557812929)
        p_vap = 6.1078 * math.pow(10,((7.5*temperature) / (temperature + 273.15)))
        p_dry = air_pressure - p_vap * humidity
        Rd = 287.05 # J/(kg*degK)
        Rvap = 461.50 # J/(kg*degK)
        return ( p_dry / (Rd * (temperature+273.15)) +
                 p_vap * humidity / (Rvap * (temperature+273.15)) )
        
    def acceleration_diff_equn(self, time, velocity): 
        # Define list to store all acceleration compoents
        comm_factor = 1 / (self.mass + (self.wheel_j / (self.wheel_rad**2)))
        accelerations = []
        # Motor torque compoent
        accelerations.append(comm_factor * self.curr_motor_torque / self.wheel_rad)
        # Coulombic friction compoent
        accelerations.append(comm_factor * -1.0 * self.c_coulombic *
                             numpy.sign(velocity) / self.wheel_rad)
        # Viscous damping component
        accelerations.append(comm_factor * -1.0 * self.c_viscous * velocity *
                             numpy.sign(velocity) / (self.wheel_rad**2))
        # Mechanical brakes compoenent
        accelerations.append(comm_factor * self.curr_brake_torque / self.wheel_rad)
        # Aerodynamic drag compoenent
        accelerations.append(comm_factor * -0.5 * (velocity**2.0)  *
                             self.curr_air_density * self.frontal_area * self.c_drag)
        # Gravity compoenent
        accelerations.append(comm_factor * -1 * self.A_GRAVITY * self.mass *
                             math.sin(math.atan(self.curr_road_gradient)))
        # Return superposition of all acceleration components
        return sum(accelerations)
    
    def rungeK_kutta_4(self, function, t, delta_t, y): 
        k1 = delta_t * function(t, y);
        k2 = delta_t * function(t+delta_t/2.0, y+k1/2.0);
        k3 = delta_t * function(t+delta_t/2.0, y+k2/2.0);
        k4 = delta_t * function(t+delta_t, y+k3);
        y_next = y + (k1 + 2.0*k2 + 2.0*k3 + k4)/6.0;
        return y_next
    
    def get_front_normal_force(self, motor_torque, brake_torque):
        """Function to return the normal force on each front wheel based on half car model"""
        motor_force = motor_torque / self.wheel_rad
        braking_force = brake_torque / self.wheel_rad
        self.normal_force_front = ( (self.cog_height * (braking_force - motor_force)
                                     + self.wheelbase * self.mass * self.A_GRAVITY)
                                     / (self.acceleration[0] + self.wheelbase))          
    
    def update(self, motor_torque, brake_torque, time, road_gradient, elevation, temperature, humidity):
        self.curr_road_gradient = road_gradient
        self.curr_motor_torque = motor_torque
        self.curr_brake_torque = brake_torque
        self.curr_air_density = self.get_air_density(elevation, temperature, humidity)
        # Remove previous, previous values
        self.acceleration.pop(0)
        self.velocity.pop(0)
        # Calcualte current acceleration using 4th order runge kutta
        self.velocity.append( self.rungeK_kutta_4(
                              self.acceleration_diff_equn,
                              time,
                              self.timestep,
                              self.velocity[0]) )
        # Differentiate to find average acceleration
        self.acceleration.append((self.velocity[1] - self.velocity[0]) / self.timestep)
        # Update front normal force (single wheel):
        # self.normal_force_front = self.get_front_normal_force(motor_torque, brake_torque)
    
class Array:
    """Class for array block"""
    
    def __init__(self, area, efficiency):
        self.power = 0                   #current front array power produced, W
        self.area = area                 #area of array in m^2
        self.efficiency = efficiency
        
    def update(self, irradiance):
        self.power = self.area * irradiance * self.efficiency

class Sunlight:
    """Class for sunlight block"""
    
    TO_DEGREES = 57.2957549575
    SOLAR_CONST = 1367                   # W/m^2
    
    def __init__(self, array_angle):
        self.irradiance = 0              #curent irradiance on car, W/m^2
        self.array_angle = array_angle   #angle of array with respect to horizontal
    
    def get_declination(self, date_time):
        # Implement's Spencer's equation for declination (1971)
        n = date_time.timetuple().tm_yday # find n'th day of the year
        B = (n-1)*(360 /365)
        declination = (180/math.pi)*(0.006918 - 0.399912*cosd(B)
                      + 0.070257*sind(B) - 0.006758*cosd(2*B)
                      + 0.000907*sind(2*B) - 0.002697*cosd(3*B)
                      + 0.00148*sind(3*B))
        return declination
    
    def get_surface_azimuth(self, heading):
        return heading - 180
    
    def get_hour_angle(self, date_time):
        # The hour angle is 15 degrees per hour before or after noon,
        # morning negative, afternoon positive
        return 15 * (date_time.hour - 12)
    
    def get_zinuth(self, dclitn, position, hr_ang):
        # Uses formula from Solar Engineerin of Thermal Processes (2013) Duffie et al
        lat = position.latitude
        return acosd( cosd(lat) * cosd(dclitn) * cosd(hr_ang)
                         + sind(lat) * sind(dclitn) )
    
    def get_angle_of_incidence(self, dclitn, position, hr_ang, azimuth, gradient, array_angle):
        # Uses formula from Solar Engineerin of Thermal Processes (2013) Duffie et al
        lat = position.latitude
        slope = atand(gradient) * self.TO_DEGREES + array_angle
        return acosd( sind(dclitn) * sind(lat) * cosd(slope)
               - sind(dclitn) * cosd(lat) * sind(slope) * cosd(azimuth)
               + cosd(dclitn) * cosd(lat) * cosd(slope) * cosd(hr_ang)
               + cosd(dclitn) * sind(lat) * sind(slope) * cosd(azimuth) * cosd(hr_ang)
               + cosd(dclitn) * sind(slope) * sind(azimuth) * sind(hr_ang) )
    
    def get_beam_transmission(self, elevation, zinuth):
        a0 = 0.4237 - 0.00821 * ( 6 - (elevation/1000) )**2
        a1 = 0.5055 + 0.00595 * ( 6.5 - (elevation/1000) )**2
        k = 0.2711 + 0.01858 * ( 2.5 - (elevation/1000) )**2
        return a0 + a1 * math.exp( -k / cosd(zinuth) )
    
    def get_diffuse_transmission(self, beam_transmission):
        return 0.271 - 0.294 * beam_transmission
    
    def get_total_irradiance(self, ang_of_incidence, zinuth, beam_transmission, diffuse_transmission, cloudienss, reflectance, gradient):
        beam_irradiance = beam_transmission * self.SOLAR_CONST * (1-cloudienss)
        diffuse_irradiance = diffuse_transmission * self.SOLAR_CONST * cosd(zinuth) * (1-cloudienss)
        slope = atand(gradient) * self.TO_DEGREES + self.array_angle
        return ( beam_irradiance * cosd(ang_of_incidence) * int(cosd(ang_of_incidence) > 0) +
                 diffuse_irradiance * (1 + cosd(slope))/2.0 +
                 (beam_irradiance + diffuse_irradiance) * reflectance * (1 - cosd(slope))/2 )

    def update(self, position, elevation, gradient, heading, date_time, cloudienss, reflectance):
        dclitn = self.get_declination(date_time)
        hr_ang = self.get_hour_angle(date_time)
        azimuth = self.get_surface_azimuth(heading)
        zinuth = self.get_zinuth(dclitn, position, hr_ang)
        ang_of_incidence = self.get_angle_of_incidence(dclitn, position, hr_ang, azimuth, gradient, self.array_angle)
        beam_transmission = self.get_beam_transmission(elevation, zinuth)
        diffuse_transmission = self.get_diffuse_transmission(beam_transmission)
        self.irradiance =  self.get_total_irradiance(ang_of_incidence,
                                                zinuth, beam_transmission,
                                                diffuse_transmission,
                                                cloudienss, reflectance, gradient)
        
class HighVoltageLosses:
    """Class for high voltage losses block"""
    
    def __init__(self): 
        self.power = 0                 #current HV system consumption, W
        
    def update(self):
        pass

class LowVoltageLosses:
    """Class for low voltage losses block"""
    
    def __init__(self):
        self.power = 0                 #current LV system consumption, W
        
    def update(self):
        pass

class VehicleLog:
    """Class logging data after each iteration"""

    def __init__(self):
        self.date_time = []
        self.elapsed_time = []
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
        self.front_irradiance = []
        self.rear_irradiance = []
        self.front_array_power = []
        self.rear_array_power = []
        self.hv_losses = []
        self.lv_losses = []
        
class Vehicle:
    """Class to handle all highest level vehicle functions"""
    
    TIMESTEP = 0.1                        #timestep for iterataive calcs, s
    WAYPOINT_TOL = 2.5                    #tolerance for reaching waypoint, m
    
    def __init__(self, vehicle_params, start_date_time, init_waypoint):
        self.params = vehicle_params
        
        # Vehicle level state variables
        self.date_time = start_date_time
        self.elapsed_time = 0
        self.position = init_waypoint.coordinate #current poition, deg
        self.elevation = init_waypoint.elevation #current elevation above sea level, m
        self.heading = 0                         #direction of travel, deg
        self.gradient = 0                        #slope of current gradient, rise/run
        self.total_distance = 0                  #totat distance traveled, m
        self.target_speed = None
        self.cloudiness = None
        self.temperature = None
        self.reflectance = None
        self.humidity = None
        
        # Classes that represent blocks in the system model. These blocks
        # contain and maipulate other state variables
        self.curr_waypoint = init_waypoint # waypoint most recently passed
        self.next_waypoint = init_waypoint # next waypoint to drive to
        self.driver = Driver(self.params.driver_k_p,self.params.driver_k_d,
                             self.params.driver_k_i, Vehicle.TIMESTEP)           
        self.motor = Motor(self.params.mu_s_tire, self.params.mu_k_tire,
                           self.params.diam_wheel, self.params.stall_torque,
                           self.params.no_load_speed, self.params.regen_gain,
                           self.params.num_motors)
        self.mech_brakes = Brakes(self.params.mu_k_brakes, self.params.mu_s_tire,
                                  self.params.mu_k_tire, self.params.area_caliper_cyl,
                                  self.params.max_line_pressue, self.params.diam_brake_rotor,
                                  self.params.diam_wheel,
                                  self.params.num_calipers_per_wheel,
                                  self.params.num_brake_wheels)
        self.car_dynamics = CarDynamics(self.params.mass, self.params.c_viscous,
                                        self.params.c_coulombic, self.params.c_drag,
                                        self.params.diam_wheel, self.params.j_wheel,
                                        self.params.j_motor_rotor, Vehicle.TIMESTEP,
                                        self.params.frontal_area, self.params.wheelbase,
                                        self.params.cog_y)
        self.rear_array = Array(self.params.array_front_area,
                                self.params.array_efficiency)
        self.front_array = Array(self.params.array_front_area,
                                 self.params.array_efficiency)
        self.front_sunlight = Sunlight(self.params.array_front_angle)
        self.rear_sunlight = Sunlight(self.params.array_rear_angle)
        self.battery = Battery(Vehicle.TIMESTEP, self.params.initial_soc)
        self.lv_losses = LowVoltageLosses()
        self.hv_losses = HighVoltageLosses()
        
        # Logfile
        self.log = VehicleLog()
        
    def log_datapoint(self):
        self.log.date_time.append(self.date_time)
        self.log.elapsed_time.append(self.elapsed_time)
        self.log.position.append(self.position)
        self.log.elevation.append(self.elevation)
        self.log.heading.append(self.heading)
        self.log.gradient.append(self.gradient)
        self.log.total_distance.append(self.total_distance)
        self.log.battery_soc.append(self.battery.soc)
        self.log.commaned_throttle.append(self.driver.commanded_throttle)
        self.log.brake_pedal_force.append(self.driver.brake_pedal_input)
        self.log.mech_brake_torque.append(self.mech_brakes.torque)
        self.log.motor_torque.append(self.motor.torque)
        self.log.motor_power.append(self.motor.power)
        self.log.velocity.append(self.car_dynamics.velocity[1])
        self.log.acceleration.append(self.car_dynamics.acceleration[1])
        self.log.front_irradiance.append(self.front_sunlight.irradiance)
        self.log.rear_irradiance.append(self.rear_sunlight.irradiance)
        self.log.front_array_power.append(self.front_array.power)
        self.log.rear_array_power.append(self.rear_array.power)
        self.log.hv_losses.append(self.hv_losses.power)
        self.log.lv_losses.append(self.lv_losses.power)

    def update_vehicle_state(self):
        """Function to..."""
        self.front_sunlight.update(self.position, self.elevation, self.gradient,
                                   self.heading, self.date_time, self.cloudiness,
                                   self.reflectance)
        self.rear_sunlight.update(self.position, self.elevation, self.gradient,
                                  self.heading, self.date_time, self.cloudiness,
                                  self.reflectance)
        self.front_array.update(self.front_sunlight.irradiance)
        self.rear_array.update(self.rear_sunlight.irradiance)
        self.driver.update(self.car_dynamics.velocity[1],
                           self.curr_waypoint.target_speed)
        self.motor.update(self.driver.commanded_throttle, self.car_dynamics.velocity[1],
                          self.car_dynamics.normal_force_front, self.curr_waypoint.target_speed)
        self.mech_brakes.update(self.driver.brake_pedal_input,
                                self.car_dynamics.normal_force_front)
        self.car_dynamics.update(self.motor.torque, self.mech_brakes.torque,
                                 self.elapsed_time, self.gradient, self.elevation,
                                 self.temperature, self.humidity)
        self.lv_losses.update()
        self.hv_losses.update()
        self.battery.update(self.motor.power, self.rear_array.power,
                            self.front_array.power,
                            self.lv_losses.power,
                            self.hv_losses.power)
        
    def drive_to_next_waypoint(self):
        """Function to iterate at period of timestep until the vehicle has
        reached the target waypoint within the given tolerance"""
        
        # Solve for original distance, heading and graident
        dist_to_waypoint = self.position.get_distance_to(self.next_waypoint.coordinate)
        self.heading = self.position.get_direction_to(self.next_waypoint.coordinate)
        self.gradient = (self.next_waypoint.elevation - self.elevation) / dist_to_waypoint
        
        # Iterate untill below toelerance or distance to waypoint begins to increase
        while(dist_to_waypoint > Vehicle.WAYPOINT_TOL):
            self.update_vehicle_state()
            # Update distance traveled
            delta_distance = abs( self.car_dynamics.velocity[0] +
                                  self.car_dynamics.velocity[1] ) * self.TIMESTEP / 2
            self.total_distance += delta_distance
            # Update elevation 
            self.elevation += self.gradient * delta_distance            
            # Update time
            self.date_time += timedelta(milliseconds=(Vehicle.TIMESTEP*1000))
            self.elapsed_time += Vehicle.TIMESTEP
            dist_to_waypoint -= delta_distance
            self.log_datapoint()

    def update_waypoints(self, new_waypoint_data):
        self.curr_waypoint = self.next_waypoint
        self.next_waypoint = Waypoint(new_waypoint_data)
        # Update params based on next waypoint
        self.position = self.curr_waypoint.coordinate
        self.target_speed = self.curr_waypoint.target_speed
        self.cloudiness = self.curr_waypoint.cloudiness
        self.temperature = self.curr_waypoint.temperature
        self.reflectance = self.curr_waypoint.reflectance
        self.humidity = self.curr_waypoint.humidity

class VehicleSimulation:
    
    def __init__(self, vehicle_params, waypoints_file_name):
        waypoints_csv = open(waypoints_file_name, 'r')
        self.csv_reader = csv.reader(waypoints_csv)
        init_date_time_data = map(int, next(self.csv_reader))
        init_waypoint = Waypoint(map(float, next(self.csv_reader)))
        init_date_time = datetime(init_date_time_data[0], init_date_time_data[1],
                                  init_date_time_data[2], init_date_time_data[3],
                                  init_date_time_data[4], init_date_time_data[5])
        self.vehicle = Vehicle(vehicle_params,init_date_time,init_waypoint)
    
    def run(self):
        """Function to iteration across waypoints"""
        for waypoint in self.csv_reader:
            self.vehicle.update_waypoints(map(float,waypoint))
            self.vehicle.drive_to_next_waypoint()
            
        plt.subplot(6, 1, 1)
        plt.plot(self.vehicle.log.elapsed_time, self.vehicle.log.elevation,'-')
        plt.title('Waterloo Loop Ouptuts 1')
        plt.ylabel('Elevation (m)')
        
        plt.subplot(6, 1, 2)
        plt.plot(self.vehicle.log.elapsed_time, self.vehicle.log.commaned_throttle, '-')
        plt.ylabel('Throttle')
        
        plt.subplot(6, 1, 3)
        plt.plot(self.vehicle.log.elapsed_time, self.vehicle.log.velocity, '-')
        plt.plot([0,500], [14,14], '-')
        plt.ylabel('Velocity (m/s)')
        
        plt.subplot(6, 1, 4)
        plt.plot(self.vehicle.log.elapsed_time, self.vehicle.log.motor_power,'-')
        plt.ylabel('Motor Power (W)')
        
        plt.subplot(6, 1, 5)
        plt.plot(self.vehicle.log.elapsed_time, self.vehicle.log.rear_array_power, '-')
        plt.ylabel('Rear Array Power (W)')
        
        plt.subplot(6, 1, 6)
        plt.plot(self.vehicle.log.elapsed_time, self.vehicle.log.battery_soc, '-')
        plt.xlabel('Time (s)')
        plt.ylabel('Battery SOC (J)')
        
        plt.show()
                
        