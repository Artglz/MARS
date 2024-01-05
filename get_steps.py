import serial
import time
import struct
from os import system, name
from math import pi, cos, sin
import csv

ser = serial.Serial('COM5', 115200, timeout=0)

COMMAND_PACKET_SIZE = 22
SENSORS_PACKET_SIZE = 103


command = bytearray([0] * COMMAND_PACKET_SIZE)
sensors = bytearray([0] * SENSORS_PACKET_SIZE)
acc = [0 for x in range(6)]
gyro = [0 for x in range(6)]
magnetic_field = [0 for x in range(12)]
proximity = [0 for x in range(16)]
distance_cm = 0
mic_volume = [0 for x in range(8)]
left_steps = [0 for x in range(4)]
right_steps = [0 for x in range(4)]
battery_raw = 0
tv_remote_data = 0
selector = 0
button_state = 0
demo_state = 0
encoder_resolution = 1000
wheel_radius = 0.0205
wheel_separation_distance = 0.053
theta = 0
x=0
y=0
left_encoder_prev = 0
right_encoder_prev = 0

def clear():
    # for windows 
    if name == 'nt': 
        _ = system('cls') 
    # for mac and linux(here, os.name is 'posix') 
    else: 
        _ = system('clear') 

# Set initial values for actuators
command[0] = 0xF8;	# -0x08: get all sensors 
command[1] = 0xF7;	# -0x09: set all actuators
command[2] = 0;		# Settings: do not calibrate IR, disable onboard OA, set motors speed
command[7] = 0x3F;	# lEDs
command[8] = 0;		# LED2 red
command[9] = 0;		# LED2 green
command[10] = 0;	# LED2 blue
command[11] = 0;	# LED4 red	
command[12] = 0;	# LED4 green
command[13] = 0;	# LED4 blue
command[14] = 0;	# LED6 red
command[15] = 0;	# LED6 green
command[16] = 0;	# LED6 blue
command[17] = 0;	# LED8 red
command[18] = 0;	# LED8 green
command[19] = 0;	# LED8 blue
command[20] = 0;	# speaker
command[21] = 0;	# End delimiter

# Enable magnetometer
magnetometer_enable_message = struct.pack(">bbb", -0x0F, 1, 0)
ser.write(magnetometer_enable_message)
reply = ser.read()
while len(reply) < 1:
    reply += ser.read()
time.sleep(0.1)

prev_time = time.time()
csv_filename = "robot_steps.csv"
with open(csv_filename, mode='w', newline='') as csv_file:
    csv_writer = csv.writer(csv_file)
    
    # Write header to the CSV file
    csv_writer.writerow(['Left', 'Right', 'PLeft', 'PRight'])
    start = time.time()

    while(1):	

        ser.write(command)
        sensors = ser.read()
        while len(sensors) < SENSORS_PACKET_SIZE:
            sensors += ser.read()
        
        # Motors steps
        left_steps = struct.unpack("<h", struct.pack("<BB", sensors[79], sensors[80]))[0]
        right_steps = struct.unpack("<h", struct.pack("<BB", sensors[81], sensors[82]))[0]



        csv_writer.writerow([left_steps, right_steps, left_encoder_prev, right_encoder_prev])
        left_encoder_prev = left_steps
        right_encoder_prev = right_steps
        print(left_steps, left_encoder_prev)
        def set_motor_speeds(left_speed, right_speed):
            command[3] = struct.unpack('<BB', struct.pack('<h', left_speed))[0]  # left motor LSB
            command[4] = struct.unpack('<BB', struct.pack('<h', left_speed))[1]  # left motor MSB
            command[5] = struct.unpack('<BB', struct.pack('<h', right_speed))[0]  # right motor LSB
            command[6] = struct.unpack('<BB', struct.pack('<h', right_speed))[1]  # right motor MSB
            

        time_diff = time.time() - start
        if time_diff < 12.0 and time_diff > 2.0:
            set_motor_speeds(600, 600)
        elif time_diff < 13.5 and time_diff > 12.0:
            set_motor_speeds(-200,200)
        elif time_diff < 22.0 and time_diff > 13.5:
            set_motor_speeds(600, 600)
        if time_diff > 22.0:
            break
   
ser.close()
