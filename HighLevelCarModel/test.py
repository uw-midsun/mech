from vehicle import VehicleConstants
from vehicle import Coordiante

test_coord1 = Coordiante(37.455926, -122.065457)
test_coord2 = Coordiante(37.509186, -122.060822)
test_coord3 = Coordiante(56.317013, -117.465206)
test_coord4 = Coordiante(56.335315, -117.506920)

print test_coord1.get_distance_to(test_coord2)
print test_coord1.get_direction_to(test_coord2)
print
print test_coord3.get_distance_to(test_coord4)
print test_coord3.get_direction_to(test_coord4)
print
print test_coord1.get_distance_to(test_coord4)
print test_coord1.get_direction_to(test_coord4)


 