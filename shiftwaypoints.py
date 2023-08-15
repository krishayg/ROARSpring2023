import math
import numpy as np
# Example list of waypoints (each waypoint is a tuple with x, y, and z coordinates)
data = np.loadtxt("ROAR/configurations/major_center_waypoints_1LAPONLY_righted.txt", delimiter=',')
x, y, z, a, b, c = data.T
waypoints = np.column_stack((x, y, z, a, b, c))
shifted_waypoints=waypoints.copy()
# Amount to shift the waypoints to the right
shift_amount = 10  # Adjust this value as needed

# Function to calculate the new position after shifting
def shift_point(point, shift_vector):
    x, y, z = point
    new_x = x + shift_vector[0]
    new_y = y + shift_vector[1]
    new_z = z + shift_vector[2]
    return np.array([new_x, new_y, new_z])

# Shift the waypoints to the right based on track segment orientation
#shifted_waypoints = []

for i in range(len(waypoints)):
    prev_point = waypoints[i - 1]
    current_point = waypoints[i]
    
    # Calculate the vector between current and previous waypoints
    vector = (current_point[0] - prev_point[0], current_point[1] - prev_point[1], current_point[2] - prev_point[2])
    
    # Calculate the perpendicular shift vector
    shift_vector = (-vector[1], vector[0], vector[2])
    
    # Normalize the shift vector and scale by the shift_amount
    magnitude = math.sqrt(sum(v ** 2 for v in shift_vector))
    normalized_shift_vector = [v * shift_amount / magnitude for v in shift_vector]
    
    # Calculate the new position after shifting
    new_waypoint = shift_point(current_point[0:3], normalized_shift_vector)
    #print(new_waypoint,waypoints[i][3:],shifted_waypoints[i])
    shifted_waypoints[i]=np.concatenate((new_waypoint, waypoints[i][3:]))
    #shifted_waypoints.append(new_waypoint)

# Print the shifted waypoints

waypoints_file=open("ROAR/configurations/major_center_waypoints_1LAPONLY_rightedTEMP.txt","w")
for i in shifted_waypoints:
    waypoints_file.write(','.join(i.astype(str))+'\n')