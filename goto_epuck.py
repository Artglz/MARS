import logging
import time
import csv

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.log import LogConfig
from cflib.positioning.position_hl_commander import PositionHlCommander
from cflib.utils import uri_helper

URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')
position_estimate = [0, 0]

logging.basicConfig(level=logging.ERROR)

def log_pos_callback(timestamp, data, logconf):
    print(data)
    global position_estimate
    position_estimate[0] = data['stateEstimate.x']
    position_estimate[1] = data['stateEstimate.y']

def read_csv(file_path):
    x_values = []
    y_values = []
    with open(file_path, 'r') as csv_file:
        csv_reader = csv.reader(csv_file)
        next(csv_reader)  # Skip header
        for row in csv_reader:
            x_values.append(float(row[0])) 
            y_values.append(float(row[1]))  
    return x_values, y_values

if __name__ == '__main__':
    # Initialize the low-level drivers
    cflib.crtp.init_drivers()

    csv_file_path = 'robot_trajectory.csv'
    e_puck_x_values, e_puck_y_values = read_csv(csv_file_path)

    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
        logconf = LogConfig(name='Position', period_in_ms=10)
        logconf.add_variable('stateEstimate.x', 'float')
        logconf.add_variable('stateEstimate.y', 'float')
        scf.cf.log.add_config(logconf)
        logconf.data_received_cb.add_callback(log_pos_callback)
        logconf.start()
        with PositionHlCommander(
                scf,
                x=0.0, y=0.0, z=0.0,
                default_velocity=0.3,
                default_height=0.5,
                controller=PositionHlCommander.CONTROLLER_PID) as pc:

            #for x_target, y_target in zip(e_puck_x_values, e_puck_y_values):
            pc.go_to(0.5, 0.5, 0.5)

        logconf.stop()
