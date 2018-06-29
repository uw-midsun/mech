#!/usr/bin/env python

""" Constants related to specific vehicles

Module contains a dictionary for each vehicle that may be subjected to the
model. Key value pairs are loaded into the VehicleConstants class in the model. 

All units are as follows unless otherwise specified:
    Distance: meters (m)
    Mass: kilograms (kg)
    Time: seconds (s)
    Angles: radians (rad)
    Temperature: degrees celcius (degC)

Midnight Sun Solar Car Team 2018
University of Waterloo
"""

__author__ = "Devon Copeland"
__copyright__ = "Copyright 2018, Midnight Sun Solar Car Team"

msxii = {
    """Weight Distribution:"""
    "mass": 550,                #curb weight, kilograms
    "cog_x": 0,                 #center of gravity in lateral direction, m
    "cog_y": 0.52,              #center of gravity in vertical direction, m
    "cog_z": 1.52,              #center of gravity in longitudinal direction, m
    "j_wheel": 2,            #polar mass moment of inertia of wheel, kg/m^2
    "j_motor_rotor": 1,         #polar mass moment of inertia of motor, kg/m^2
    
    """Dimensions:"""
    "wheelbase": 2.6,           #wheelbase, meters
    "diam_wheel": 0.53,         #wheel diameter, meters
    "area_frontal": 0.2,        #total frontal area, meters^2
    
    """Brakes:"""
    "area_caliper_cyl": 0.0006387084, #caliper cylinder area, meters^2,
    "area_master_cyl": 0.0002838704, #caliper cylinder area, meters^2,
    "mu_k_brakes": 0.2,         #coeficient of drag, unitless
    "max_line_pressue": 10^7,   #pressure, Pa, assumes 400N pedal force with 7:1 pedal ratio
    "diam_brake_rotor": 0.53,   #wheel diameter, meters
    "num_calipers_per_wheel": 2,          #number or brake calipers
    
    """Motors:"""
    "driver_k_p": .1,
    "driver_k_d": 0,
    "driver_k_i": 0, 
    
    """Motors:"""
    "stall_torque": 135,        #stall torque of motor, Nm
    "no_load_speed": 1700,      #no load speed, rpm
    "regen_gain": 0.01,         #gain applied to angular velocity to get 
    
    """Coefficients:"""
    "c_drag": 0.2,              #coeficient of drag, unitless
    "c_coulombic": 0.2,         #coeficient of coulombic friction (for entire vehicle), unitless
    "c_viscous": 0.2,           #coeficient of viscous damping (for entire vehicle), unitless
    "mu_s_tire": 0.7,           #tire coeficient of static frction, unitless
    "mu_k_tire": 0.6,           #tire coeficient of kinetic frction, unitless

    """Array:"""
    "array_front_area": 1.1653, #total area, m^2 (76 cells * 0.0153328 m^2 per cell)
    "array_rear_area": 3.8332,  #total area, m^2 (250 cells * 0.0153328 m^2 per cell)
    "array_front_angle": -15,   #angle of hood, deg
    "array_rear_angle": 10,     #angle of rear pannel, deg
    "array_efficiency": 0.237   #array efficiency, unitless
}
