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
    "j_wheel": None,            #polar mass moment of inertia of wheel, kg/m^2
    "j_motor_rotor": None,      #polar mass moment of inertia of motor, kg/m^2
    
    """Dimensions:"""
    "wheelbase": 2.6,           #wheelbase, meters
    "diam_wheel": 0.53,         #wheel diameter, meters
    "diam_brake_rotor": 0.53,         #wheel diameter, meters
    "area_frontal": 0.2,        #total frontal area, meters^2
    "area_caliper_cyclinder": NaN,  #brake pad constact area, meters^2,
                                #   http://www.wilwood.com/BrakePads/BrakePadsProd.aspx?itemno=150-4091K
    
    """Coefficients:"""
    "cd": 0.2,                  #coeficient of drag, unitless
    "c_coulombic": 0.2,         #coeficient of coulombic friction (for entire
                                #    vehicle), unitless
    "c_viscous": 0.2,           #coeficient of viscous damping (for entire
                                #    vehicle), unitless
    "mu_k_brakes": 0.2,         #coeficient of drag, unitless
    "cd": 0.2,                  #coeficient of drag, unitless
}