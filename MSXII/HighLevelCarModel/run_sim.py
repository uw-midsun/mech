from vehicle import *
import vehicle_data
import datetime

# Load MSXII Data
msxii_data = VehicleConstants()
msxii_data.load_vehicle(vehicle_data.msxii)

# Load Waypoint File
waypoints_file = "farson_arco.csv"

# Run Simulation
sim  = VehicleSimulation(msxii_data, waypoints_file)
sim.run()