from vehicle import *
import vehicle_data
import datetime

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
test_coord1 = Coordinate(37.455926, -122.065457)
test_coord2 = Coordinate(37.509186, -122.060822)
test_coord3 = Coordinate(56.317013, -117.465206)
test_coord4 = Coordinate(56.335315, -117.506920)
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
test_brakes = Brakes(1.0, .4, .25, 0.001, 1000000,.2, .5, 2, 2)
for i in range(0,11):
    test_brakes.update((i/10.0), 1500)
    print ("%d0%% brake: %5.2f" % (i, test_brakes.torque))
print
# Torque at 50% braking is 200Nm
# Max toque before sliding is 300Nm
# Sliding torque is 187.5Nm

"""Motor Class:"""
print
print "Motor Class:"
test_motor = Motor(0.8, 0.5, 0.5, 135, 1700, -0.15, 2)
test_motor.update(0, 20, None)
print ("0%% throttle, 20m/s - Torque: %5.2f, Power: %5.2f" % (test_motor.torque, test_motor.power))
test_motor.update(0.5, 20, None)
print ("50%% throttle, 20m/s - Torque: %5.2f, Power: %5.2f" % (test_motor.torque, test_motor.power))
test_motor.update(1.0, 20, None)
print ("100%% throttle, 20m/s - Torque: %5.2f, Power: %5.2f" % (test_motor.torque, test_motor.power))
print

"""Car Dynamics Class:"""
print
print "Car Dynamics Class:"
test_car_dynamics = CarDynamics(635, 0.01, 0.1, 0.25, 0.53, 0.2, 0.1, 0.1, 3, 2.6, 0.52)
print test_car_dynamics.get_air_density(500, 20, .5)
velocity_log = []
acceleration_log = []
time_log = []
normal_force_front_log = []
for i in range(1,2001):
    test_car_dynamics.update(200, 0, i*0.1, 0.0875, 500, 20, .5)
    velocity_log.append(test_car_dynamics.velocity[0])
    acceleration_log.append(test_car_dynamics.acceleration[0])
    time_log.append(i*0.1)
    normal_force_front_log.append(test_car_dynamics.normal_force_front)

plt.subplot(3, 1, 1)
plt.plot(time_log, normal_force_front_log, 'o-')
plt.title('Car Dynamics Test Case 1')
plt.ylabel('Normal Force (N)')

plt.subplot(3, 1, 2)
plt.plot(time_log, velocity_log, '.-')
plt.ylabel('Velocity (m/s)')
plt.ylim(0,40)

plt.subplot(3, 1, 3)
plt.plot(time_log, acceleration_log, '--')
plt.xlabel('Time (s)')
plt.ylabel('Acceleration (m/s^2)')

#plt.show()

"""Sunlight Class:"""
print
print "Sunlight:"
test_sunlight = Sunlight(10)
date_time = datetime.datetime(2018, 8, 1, 11, 0, 0)
print str(date_time)
print date_time.timetuple().tm_yday
postition = Coordinate(47.6, 122.3167)
test_sunlight.update(postition, 5000, .06, 340, date_time, 1, 0.5)
print test_sunlight.irradiance 