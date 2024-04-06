# The main script for replaying augmented test bags
# and storing the planning outputs

import json
import os
import sys
import csv
import time
import random
import signal
import argparse
import subprocess
from multiprocessing import Process, Manager

from automation.auxiliary.routing import mock_routing_request
from automation.grading_metrics import acceleration, collision

try:
    from subprocess import DEVNULL  # Python 3.
except ImportError:
    DEVNULL = open(os.devnull, 'wb')

MAX_RECORD_TIME = 30

RECORDER_PATH = '/apollo/scripts/record_bag.py'
USE_CSV_ROUTING = False
OUTPUT_NAME = 'output'

# Stores output record files from simulation
TEMP_OUTPUT_PATH = '/apollo/automation/temp_record/'


def run_park_simulation(ego_routing):
    mock_routing_request.request_park_routing(ego_routing)
    time.sleep(1.0)
    record_output()

def record_output(record_time=10):
    # Start recording messages and producing perception messages
    start_record_cmd = f'cyber_recorder record -o {TEMP_OUTPUT_PATH}{OUTPUT_NAME} -a &'
    
    subprocess.Popen(start_record_cmd,
                     shell=True,
                     stdout=DEVNULL,
                     stderr=DEVNULL)

    p = subprocess.Popen(
        ['/apollo/modules/tools/perception/sunnyvale_loop_perception.bash'],
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
        # processes.append(
        #     Process(target=speeding.walk_messages,
        #             args=(output_path,),
        #             kwargs={'return_dict': oracle_results}))

    for process in processes:
        process.start()

    for process in processes:
        process.join()

    accl = oracle_results['accl']
    hardbreak = oracle_results['hardbreak']
    min_dist = oracle_results['min_dist']
    collision_states = oracle_results['collision_states']
    # min_speed = oracle_results['min_speed']
    # boundary_dist = oracle_results['boundary_dist']

    # return min_dist, boundary_dist, accl, hardbreak, collision_states

    return min_dist, accl, hardbreak, collision_states


def main():
    global OUTPUT_NAME

    # define required arguments
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '-f', '--filename',
        help='The config file path of the ego\'s routing',
        type=str
    )
    parser.add_argument(
        '-o', '--output',
        help='The name of output record',
        type=str
    )
    args = parser.parse_args()

    ego_routing = {}
    with open(args.filename, 'r') as f:
        json.load(ego_routing, f)

    if not os.path.exists(TEMP_OUTPUT_PATH):
        subprocess.run(['mkdir', TEMP_OUTPUT_PATH])

    if args.output:
        OUTPUT_NAME = args.output
    
    sim_time=time.time()
    run_park_simulation(ego_routing)
    sim_time=time.time()-sim_time

    orcle_time=time.time()
    min_dist, accl, hardbreak, collision = run_oracles()
    # min_dist, min_speed, boundary_dist, accl, hardbreak, collision = run_oracles()
    orcle_time=time.time()-orcle_time

    # if min_dist == set() or len(lanes_only) == 0:
    #     print(None)
    # else:
    print(min_dist, accl, hardbreak, collision, sim_time, orcle_time, sep="\n")


if __name__ == '__main__':
    main()
