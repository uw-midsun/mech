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
from numpy import sign

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
        """Funtion to compute the direction to point to target coordinate.
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
    
    def __init__(self, lat, longi, elev, targ_vel, cloud, temp, press, reflect, humid):
        self.coordinate = Coordiante(lat, longi)  #next waypoint poition, deg  
        self.elevation = elev            #current elevation above sea level, m
        self.target_velocity = targ_vel  #intended velocity at waypoint, m/s
        self.cloudiness = cloud          #factor of irradiance blocked by clouds, %
        self.temperature = temp          #current temperature, degC
        self.air_pressure = press        #current air pressure, Pa
        self.reflectance = reflect       #current groudn refectance, 
        self.humidity = humid            #relative humidity, %

class Battery:
    """Class to handle all battery level funcitons"""
    
    def __init__(self, input_timestep, initial_soc):
        self.timestep = input_timestep
        self.soc = initial_soc           #current battery pack state of charge, J
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
        self.soc += ( (self.net_power[0] +
                      self.net_power[1]) * self.timestep / 2 )
 
class Driver:
    """Class for driver block"""
    
    def __init__(self, kP, kI, kD, timestep):
        self.kP = kP
        self.kI = kI
        self.kD = kD
        self.timestep = timestep
        self.error_integrator = 0
        self.velocity_error = [0, 0]     #fixed length fifo to store last 2 velocity error terms
        self.commanded_throttle = NaN    #current commanded throttle, %
        self.brake_pedal_input = NaN     #current brake pedal force, N
        
    def update(self, curr_velocity, target_velocity):
        """Compuptes control signal and adjusts commended throttle and brake
        pedal input accordingly. Both are values from 0 to 1 represeting the
        amount the pedal is depressed. This will be mapped to torques in the
        brakes and motor classes"""
        self.velocity_error.pop(0)
        self.velocity_error.append(target_velocity-curr_velocity)
        self.error_integrator += ( (self.velocity_error[0] +
                                   self.velocity_error[1]) * self.timestep / 2 )
        # Compute control signal based on PID values
        control_signal = ( self.velocity_error[1] * self.kP +
                           self.error_integrator * self.kI +
                           (self.velocity_error[1] - self.velocity_error[0]) * self.kD )
        
        # Compute pedal commands based on control signal
        if control_signal > 1:
            self.commanded_throttle = 1
            self.brake_pedal_input = 0
        elif control_signal >= 0:
            self.commanded_throttle = control_signal
            self.brake_pedal_input = 0
        elif control_signal >= -1:
            self.commanded_throttle = 0
            self.brake_pedal_input = abs(control_signal)
        else:
            self.commanded_throttle = 0
            self.brake_pedal_input = 1
    
class Brakes:
    """Class for brakes block"""
    
    def __init__(self, mu_k_brakes, mu_s_tire, area_caliper_cyl, max_line_pressue,
                 diam_brake_rotor, diam_wheel, num_brake_calipers):
        self.mu_k_brakes = mu_k_brakes
        self.mu_s_tire = mu_s_tire
        self.area_caliper_cyclinder = area_caliper_cyl
        self.max_line_pressue = max_line_pressue
        self.diam_brake_rotor = diam_brake_rotor
        self.diam_wheel = diam_wheel
        self.num_brake_calipers = num_brake_calipers
        self.torque = 0          #current motor torque, Nm
        
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
    
    TO_RAD_PER_SECOND = 0.1047197177
    
    def __init__(self, mu_s_tire, mu_k_tire, diam_wheel, stall_torque,
                 no_load_speed, regen_gain):
        self.torque = 0          #current motor torque, Nm
        self.power = 0           #current motor consumption, W
        self.mu_s_tire = mu_s_tire
        self.mu_k_tire = mu_k_tire
        self.diam_wheel = diam_wheel
        self.stall_torque = stall_torque
        self.no_load_speed = no_load_speed
        self.regen_gain = regen_gain
        
    def get_torque_from_throttle(self, commanded_throttle, curr_rpm):
        """Function to map throttle to motor torque. Assume linear model
           untill torque speed curve can be input"""
        torque_vs_speed_slope = - (self.stall_torque / self.no_load_speed)
        speed_adjusted_max_torque = stall_torque + torque_vs_speed_slope * curr_velocity
        return speed_adjusted_max_torque * commanded_throttle

    def get_drive_efficiency(self, curr_rpm):
        """Function to get motor drive efficiency from torque and speed"""
        return 0.8 #[IMPROVEMENT] Set as constant for now
    
    def get_regen_efficiency(self, curr_rpm):
        """Function to get motor regen braking efficiency from torque and speed"""
        return 0.8 #[IMPROVEMENT] Set as constant for now
        
    def update(self, commanded_throttle, curr_velocity, single_wheel_normal_force_rear):
        """Function to update motor toque"""
        curr_rpm = curr_velocity * 60 / (math.pi * self.diam_wheel)
        if commanded_throttle == 0: # Check for regen braking, use linear model 
            self.torque = self.regen_gain * curr_rpm
            self.power = self.torque * curr_rpm * Motor.TO_RAD_PER_SECOND * get_regen_efficiency(curr_rpm)
        else: 
            self.torque = get_torque_from_throttle(commanded_throttle, curr_rpm)
            self.power = self.torque * curr_rpm * Motor.TO_RAD_PER_SECOND * get_drive_efficiency(curr_rpm)
    
        #[IMPROVEMENT] Could add logic to check for tire slipping here
    
class CarDynamics:
    """Class for car dynamics block"""
    
    TO_RADIANS = 0.0174533
    A_GRAVITY = 9.81
    
    def __init__(self, mass, c_viscous, c_coulombic, c_drag, wheel_rad, wheel_j,
                 motor_rotor_j):
        # lower indices are most recent
        self.velocity = [0,0]              # velocity list, m/s
        self.acceleration = [0,0]          # acceleration list, m/s^2
        self.mass = mass
        self.c_viscous = c_viscous
        self.c_coulombic = c_coulombic
        self.c_drag = c_drag
        self.wheel_rad = wheel_rad
        self.wheel_j = wheel_j
        self.motor_rotor_j = motor_rotor_j
        self.curr_road_gradient = NaN
        self.curr_motor_torque = NaN
        self.curr_brake_torque = NaN
        self.curr_air_density = NaN
        
    def get_air_density(self, elevation, temperature, humidity, air_pressure):
        # Funtion to determine air density based on temperature, pressure
        # and humidity. Uses partial pressures of dry air and water vapour
        # https://wahiduddin.net/calc/density_altitude.htm
        p_vap = 610.78 * 10^((7.5*temperature) / (temperature + 273.15))
        p_dry = air_pressure - p_vap * humidity
        Rd = 287.05 # J/(kg*degK)
        Rvap = 461.50 # J/(kg*degK)
        return ( p_dry / (Rd * (temperature+273.15)) +
                 p_vap * humidity / (Rvap * (temperature+273.15)) )
        
    def acceleration_diff_equn(self, time, velocity): 
        # Define list to store all acceleration compoents
        accelerations = []
        # Motor torque compoent
        accelerations.append(self.curr_motor_torque /
                             (self.wheel_rad  * self.mass * self.wheel_j))
        # Coulombic friction compoent
        accelerations.append(-self.c_coulombic * numpy.sign(velocity) /
                             (self.wheel_rad  * self.mass * self.wheel_j))
        # Viscous damping component
        accelerations.append(-self.c_viscous * velocity * numpy.sign(velocity) / 
                             (self.mass * self.wheel_j))
        # Mechanical brakes compoenent
        accelerations.append(-self.curr_brake_torque /
                             (self.wheel_rad**2  * self.mass * self.wheel_j))
        # Aerodynamic drag compoenent
        accelerations.append(self.curr_air_density *
                             self.c_drag * self.frontal_area * velocity**2 /
                             (self.mass * self.wheel_j))
        # Gravity compoenent
        accelerations.append(self.A_GRAVITY *
                             math.sin(self.curr_road_gradiant*self.TO_RADIANS) /
                             self.wheel_j)
        # Return superposition of all acceleration components
        return sum(accelerations)
    
    def rungeK_kutta_4(self, function, t, delta_t, y): 
        k1 = delta_t * function(t, y);
        k2 = delta_t * function(t+delta_t/2, y+k1/2);
        k3 = delta_t * function(t+delta_t/2, y+k2/2);
        k4 = delta_t * function(t+delta_t, y+k3);
        y_next = y + (k1 + 2*k2 + 2*k3 + k4)/6;
        return y_next 
    
    def update(self, motor_torque, brake_torque, time, road_gradient, elevation, temperature, humidity, air_pressure):
        self.road_gradient = road_gradient
        self.curr_motor_torque = motor_torque
        self.curr_brake_torque = brake_torque
        self.curr_air_density = self.get_air_density(elevation, temperature, humidity)
        # Remove previous, previous values
        self.accceleration.pop(0)
        self.velocity.pop(0)
        # Calcualte current acceleration using 4th order runge kutta
        self.acceleration.append( self.rungeK_kutta_4(
                                        self.acceleration_diff_equn,
                                        time,
                                        self.timestep,
                                        self.velocity[1]) )
        # Integerate using trapazoidal rule to find next velocity
        self.velocity.append( self.velocity[0] + ( self.acceleration[0] +
                                                   self.acceleration[1] * self.timestep / 2 ))
    
class Array:
    """Class for array block"""
    
    def __init__(self, area, efficiency):
        self.power = NaN                 #current front array power produced, W
        self.area = area                 #area of array in m^2
        self.efficiency = efficiency
        
    def update(self, irradiance):
        self.power = self.area * irradiance * self.efficiency

class Sunlight:
    """Class for sunlight block"""
    
    TO_DEGREES = 57.2957549575
    SOLAR_CONST = 1367                   # W/m^2
    
    def __init__(self, array_angle):
        self.irradiance = NaN            #curent irradiance on car, W/m^2
        self.array_angle = array_angle   #angle of array with respect to horizontal
    
    def get_declination(self, date_time):
        # Implement's Spencer's equation for declination (1971)
        n = date_time.timetuple().tm_day # find n'th day of the year
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
        slope = atand(gradient) * TO_DEGREES + array_angle
        return acosd( math.sin(dclitn) * sind(lat) * cosd(slope)
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
    
    def get_total_irradiance(self, ang_of_incidence, zinuth, beam_transmission, diffuse_transmission, cloudienss, reflectance):
        beam_irradiance = beam_transmission * SOLAR_CONST * cloudienss
        diffuse_irradiance = diffuse_transmission * SOLAR_CONST * cosd(zinuth) * cloudienss
        slope = atand(gradient) * TO_DEGREES + array_angle
        return ( beam_irradiance * cosd(ang_of_incidence) * numpy.heaviside(cosd(ang_of_incidence),0) +
                 diffuse_irradiance * (1 + cosd(slope))/2 +
                 (beam_irradiance + diffuse_irradiance) * reflectance * (1 - cosd(slope))/2 )

    def update(self, position, elevation, gradient, heading, date_time, cloudienss, reflectance):
        dclitn = self.get_declination(date_time)
        hr_ang = self.get_hour_angle(date_time)
        azimuth = self.get_surface_azimuth(heading)
        zinuth = self.get_zinuth(dclitn, position, hr_ang)
        ang_of_incidence = self.get_angle_of_incidence(dclitn, position, hr_ang, azimuth, gradient, array_angle)
        beam_transmission = get_beam_transmission(elevation, zinuth)
        diffuse_transmission = get_diffuse_transmission(beam_transmission)
        self.irradiance =  get_total_irradiance(ang_of_incidence,
                                                zinuth, beam_transmission,
                                                diffuse_transmission,
                                                cloudienss, reflectance)
        
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
        self.front_irradiance = []
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
        self.front_sunlight = Sunlight()
        self.rear_sunlight = Sunlight()
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
        self.log.front_irradiance.append(self.front_sunlight.irradiance)
        self.log.rear_irradiance.append(self.rear_sunlight.irradiance)
        self.log.front_array_power.append(self.front_array.power)
        self.log.rear_array_power.append(self.rear_array.power)
        self.log.hv_losses.append(self.hv_losses.power)
        self.log.lv_losses.append(self.lv_losses.power)

    def update_vehicle_state(self):
        """Function to..."""
        self.sunlight.update(self.position, self.elevation, self.gradient,
                             self.heading, self.date_time, self.cloudiness,
                             self.reflectance)
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