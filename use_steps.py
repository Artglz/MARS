import csv
from math import sin, cos, radians
import math
csv_filename = "robot_steps.csv"
output_csv_filename = "output_positions.csv"

wheel_radius = 0.0205
wheel_separation_distance = 0.0421
theta = 0
x = 0
y = 0

with open(csv_filename, mode='r') as csv_file:
    csv_reader = csv.reader(csv_file)
    header = next(csv_reader)  # Skip header

    positions_data = []  # List to store positions data

    index = 2
    for row in csv_reader:
        
        left_steps, right_steps, left_encoder_prev, right_encoder_prev = map(int, row)
        
        # Perform your equation or calculations here
        left_displacement = (left_steps - left_encoder_prev)
        right_displacement = (right_steps - right_encoder_prev)
        
        # Calculate linear and angular displacement in meters and radians
        linear_distance = (left_displacement + right_displacement) * (2 * math.pi * wheel_radius) / (2 * 1000)
        angular_distance = (right_displacement - left_displacement) * (2 * math.pi) / (1000 * wheel_separation_distance) 
        '''
        linear_velocity = (left_displacement + right_displacement) / (2.0)
        angular_velocity = (right_displacement - left_displacement) / wheel_separation_distance
        '''
        theta += angular_distance
        x += linear_distance * cos(radians(theta)) 
        y += linear_distance * sin(radians(theta))
        print(index, ':', x, y)
        index += 1
        positions_data.append([x, y])

with open(output_csv_filename, mode='w', newline='') as output_csv_file:
    csv_writer = csv.writer(output_csv_file)
    
    # Write header to the new CSV file
    csv_writer.writerow(['X', 'Y'])

    # Write positions data to the new CSV file
    csv_writer.writerows(positions_data)

print(f"Positions data saved to {output_csv_filename}")