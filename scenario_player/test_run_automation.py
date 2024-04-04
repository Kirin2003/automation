# The main script for replaying augmented test bags
# and storing the planning outputs

import os
import sys
import csv
import time
import random
import signal
import argparse
import subprocess
from multiprocessing import Process, Manager

from automation.auxiliary.routing import send_routing_request
from automation.grading_metrics import acceleration, speeding, collision

from cyber.python.cyber_py3 import cyber
from cyber.python.cyber_py3 import cyber_time
from modules.routing.proto import routing_pb2

try:
    from subprocess import DEVNULL  # Python 3.
except ImportError:
    DEVNULL = open(os.devnull, 'wb')

MAX_RECORD_TIME = 60

RECORDER_PATH = '/apollo/scripts/record_bag.py'
USE_CSV_ROUTING = False
OUTPUT_NAME = 'output'

# Stores output record files from simulation
TEMP_OUTPUT_PATH = '/apollo/automation/temp_record/'


def get_args():
    # define required arguments
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '-rv', '--routing_value',
        help='The x and y values for start and end point of nevigation',
        type=str
    )
    parser.add_argument(
        '-rc', '--routing_csv',
        help='The path of the csv file containing the routing information',
        type=str
    )
    parser.add_argument(
        '-o', '--output',
        help='The name of output record',
        type=str
    )
    args = parser.parse_args()

    return args


def get_routing(args):
    # Obtain the routing data from csv file if provided
    # If so, the script sends routing requests based on
    # the content from routing_list
    routing_list = []
    if args.routing_csv:
        with open(args.routing_csv, mode='r') as routing_csv:
            csv_reader = csv.DictReader(routing_csv)
            for i, routing in enumerate(csv_reader):
                routing_list.append({
                    'init_x': routing['init_x'],
                    'init_y': routing['init_y'],
                    'dest_x': routing['dest_x'],
                    'dest_y': routing['dest_y']
                })
    return routing_list



    


def run_simulation(routing_list, init_x, init_y, dest_x, dest_y):
    # Make sure the index of routing does not exceed
    # the length of routing_list
    p = subprocess.Popen(
        ['/apollo/modules/tools/perception/sunnyvale_loop_perception.bash'],
        stdout=DEVNULL,
        stderr=DEVNULL)
    
    time.sleep(5.0)

    if (len(routing_list) > 0):
        routing_index = random.randrange(0, len(routing_list))
        # routing_index = 3
        routing_info = routing_list[routing_index]
        init_x, init_y, dest_x, dest_y = \
            float(routing_info['init_x']), float(routing_info['init_y']), \
            float(routing_info['dest_x']), float(routing_info['dest_y'])
    
    send_routing_request.request_routing(
       init_x, init_y, dest_x, dest_y, verbose=False)
    time.sleep(1)
    # Start recording messages and producing perception messages
    start_record_cmd = f'cyber_recorder record -o {TEMP_OUTPUT_PATH}{OUTPUT_NAME} -a &' 
       
    subprocess.Popen(start_record_cmd,
                     shell=True,
                     stdout=DEVNULL,
                     stderr=DEVNULL)

    
    # Wait for record time
    time.sleep(MAX_RECORD_TIME)
    # Stop recording messages and producing perception messages
    stop_record_cmd = f'python3 /apollo/scripts/record_bag.py --stop --stop_signal SIGINT > /dev/null 2>&1'
    subprocess.run(stop_record_cmd, shell=True)
    time.sleep(2)

    try:
        os.kill(p.pid, signal.SIGINT)
        p.kill()
    except OSError:
        print("stopped")

def run_parking_simulation(routing_list, init_x, init_y, dest_x, dest_y):
    # Make sure the index of routing does not exceed
    # the length of routing_list
    p = subprocess.Popen(
        ['/apollo/modules/tools/perception/sunnyvale_loop_perception.bash'],
        stdout=DEVNULL,
        stderr=DEVNULL)
    
    time.sleep(5.0)

    cyber.init()
    node = cyber.Node("mock_routing_requester")
    sequence_num = 0

    routing_request = routing_pb2.RoutingRequest()

    routing_request.header.timestamp_sec = cyber_time.Time.now().to_sec()
    routing_request.header.module_name = 'routing_request'
    routing_request.header.sequence_num = sequence_num
    sequence_num = sequence_num + 1

    waypoint = routing_request.waypoint.add()

    waypoint.pose.x = 587191.89
    waypoint.pose.y = 4141422.39


    waypoint = routing_request.waypoint.add()
    waypoint.pose.x = 587215.07
    waypoint.pose.y = 4141416.96

    routing_request.parking_info.parking_space_id = "11545"
    routing_request.parking_info.parking_point.x = 587201.80
    routing_request.parking_info.parking_point.y = 4141412.56
    routing_request.parking_info.parking_point.z = 0.0

    writer = node.create_writer('/apollo/routing_request',
                                routing_pb2.RoutingRequest)
    time.sleep(2.0)
    
    writer.write(routing_request)
    time.sleep(2.0)

    # Start recording messages and producing perception messages
    start_record_cmd = f'cyber_recorder record -o {TEMP_OUTPUT_PATH}{OUTPUT_NAME} -a &' 
   
    subprocess.Popen(start_record_cmd,
                     shell=True,
                     stdout=DEVNULL,
                     stderr=DEVNULL)

    
    # Wait for record time
    time.sleep(MAX_RECORD_TIME)
    # Stop recording messages and producing perception messages
    stop_record_cmd = f'python3 /apollo/scripts/record_bag.py --stop --stop_signal SIGINT > /dev/null 2>&1'
    subprocess.run(stop_record_cmd, shell=True)
    time.sleep(2)

    try:
        os.kill(p.pid, signal.SIGINT)
        p.kill()
    except OSError:
        print("stopped")

def run_oracles():
    target_output_names = []

    all_output_names = os.listdir(TEMP_OUTPUT_PATH)
    all_output_names.sort()

    for name in all_output_names:
        if name.startswith(f'{OUTPUT_NAME}.'):
            target_output_names.append(name)

    processes = []
    manager = Manager()
    oracle_results = manager.dict()

    for output_name in target_output_names:        # run checks on each output
        output_path = f'{TEMP_OUTPUT_PATH}{output_name}'

        processes.append(
            Process(target=acceleration.walk_messages,
                    args=(output_path, 4),
                    kwargs={'return_dict': oracle_results}))
        processes.append(
            Process(target=acceleration.walk_messages,
                    args=(output_path, -4),
                    kwargs={'return_dict': oracle_results}))
        processes.append(
            Process(target=collision.walk_messages,
                    args=(output_path,),
                    kwargs={'return_dict': oracle_results}))
        processes.append(
            Process(target=speeding.walk_messages,
                    args=(output_path,),
                    kwargs={'return_dict': oracle_results}))

    for process in processes:
        process.start()

    for process in processes:
        process.join()

    accl = oracle_results['accl']
    hardbreak = oracle_results['hardbreak']
    min_dist = oracle_results['min_dist']
    collision_states = oracle_results['collision_states']
    min_speed = oracle_results['min_speed']
    traveled_lanes = oracle_results['traveled_lanes']
    boundary_dist = oracle_results['boundary_dist']

    return min_dist, traveled_lanes, min_speed, boundary_dist, accl, hardbreak, collision_states


def main():
    run_nums = 0
    
    global OUTPUT_NAME
    args = get_args()       # get arguments
    # Define default routing values in case of no input from argument
    init_x, init_y = 587218.41, 4141415.99
    dest_x, dest_y = 587200.00, 4141415.99

    if args.routing_value:
        # Overwrite the default routing value since inputs are provided
        routing_inputs = args.routing_value.split(',')
        init_x, init_y = float(routing_inputs[0]), float(routing_inputs[1])
        dest_x, dest_y = float(routing_inputs[2]), float(routing_inputs[3])

    routing_list = get_routing(args)    # obtain routing information

    if not os.path.exists(TEMP_OUTPUT_PATH):
        subprocess.run(['mkdir', TEMP_OUTPUT_PATH])

    if args.output:
        OUTPUT_NAME = args.output
    
    sim_time=time.time()
    run_simulation(routing_list, init_x=587187.51, init_y=4141423.33, dest_x=587216.56, dest_y=4141416.07)
    # run_parking_simulation(routing_list, init_x, init_y, dest_x, dest_y)
    sim_time=time.time()-sim_time

    print('Done!')

    # orcle_time=time.time()
    # min_dist, all_lanes, min_speed, boundary_dist, accl, hardbreak, collision = run_oracles()
    # orcle_time=time.time()-orcle_time

    # lanes_only = ""
    # for lane in all_lanes:
        # lanes_only = lanes_only + lane[0] + " "
    # lanes_only.strip(" ")

    # if min_dist == set() or len(lanes_only) == 0:
        # print(None)
    # else:
        # print(min_dist, lanes_only, min_speed, boundary_dist,
                # accl, hardbreak, all_lanes, collision, sim_time,orcle_time,sep="\n")
        
        


if __name__ == '__main__':
    main()
