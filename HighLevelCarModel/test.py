from vehicle import *
import vehicle_data

"""Trig Functions:"""
print
print "Trig Functions:"
print "sin(30) = ", sind(30)
print "cos(60) = ", sind(60)
print "tan(45) = ", tand(45)
print "asin(0.5) = ", asind(0.5)
print "acos(0.5) = ", asind(0.5)
print "atan(1) = ", atand(1)

"""Vehicle Constants Class:"""
print
print "Vehicle Constant Class:"
msxii_data = VehicleConstants()
msxii_data.load_vehicle(vehicle_data.msxii)
print "Wheel diameter = ", msxii_data.diam_wheel

"""Coordiante Class:"""
print
print "Coordiante Class:"
test_coord1 = Coordiante(37.455926, -122.065457)
test_coord2 = Coordiante(37.509186, -122.060822)
test_coord3 = Coordiante(56.317013, -117.465206)
test_coord4 = Coordiante(56.335315, -117.506920)
print "Test Case 1:"
print test_coord1.get_distance_to(test_coord2)
print test_coord1.get_direction_to(test_coord2)
print "Test Case 2:"
print test_coord3.get_distance_to(test_coord4)
print test_coord3.get_direction_to(test_coord4)
print "Test Case 3:"
print test_coord1.get_distance_to(test_coord4)
print test_coord1.get_direction_to(test_coord4)

"""Battery Class:"""
print
print "Battery Class:"
test_battery = Battery(1)
test_battery.soc = 1000
motor_power = [50,100,-100,-100,100] #Integrate: 25, 75, 0, -100, 0
rear_array_power = [50, 50, 50, 50, 50] # Integrate: 25, 50, 50, 50, 50
front_array_power = [50, 50, 50, 50, 50] # Integrate: 25, 50, 50, 50, 50
lv_losses = [0,0,0,0,0]
hv_losses = [0,0,0,0,0]
for i in range(len(motor_power)):
    test_battery.update(motor_power[i], rear_array_power[i],
                        front_array_power[i], lv_losses[i], hv_losses[i])
    print "Net Power = ", test_battery.net_power
print "Final SOC = ", test_battery.soc #Total 75, 175, 100, 0, 100 -> final SOC = 1450

"""Driver Class:"""
print
print "Driver Class:"
test_driver = Driver(0.1, 0.1, 1, 1)
curr_velocity =    [0,  6, 12, 18, 24, 28, 37, 41]
target_velocity =  [5, 10, 15, 20, 25, 30, 35, 40]
            #error  5,  4,  3,  2,  1,  2, -2, -1
            #diff   5  -1  -1  -1  -1   1  -4   1
            #int  2.5   7  10.5 13 14.5 16 16 14.5
            #net  5.75 0.1 0.35 0.5 0.55 2.8 -2.6 2.35
for i in range(len(curr_velocity)):
    test_driver.update(curr_velocity[i], target_velocity[i])
    print ("Error = %5.2f   Throttle = %5.2f   Brake = %5.2f" %
          (test_driver.control_signal, test_driver.commanded_throttle, test_driver.brake_pedal_input))

"""Brakes Class:"""
print
print "Brakes Class:"
test_brakes = Brakes(.5, .5, .5, 0.001, 1000000,.2, .5, 4)

"""Motor Class:"""
print
print "Motor Class:"
test_motor = Motor()