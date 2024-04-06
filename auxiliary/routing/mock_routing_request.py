#!/usr/bin/env python3

# send the routing message specified in the argument
# to the channel /apollo/routing request

import sys
import time
import argparse

from cyber.python.cyber_py3 import cyber
from cyber.python.cyber_py3 import cyber_time
from modules.routing.proto.routing_pb2 import RoutingRequest


class CyberShutdown(Exception):
    pass


def create_node(node_name='automation_routing_request_node'):
    # generate node
    return cyber.Node(node_name)


def create_writer(
        node,
        channel_name='/apollo/routing_request',
        message_type=RoutingRequest):
    # generate writer
    return node.create_writer('/apollo/routing_request', message_type)


def process_message(writer, init_x, init_y, dest_x, dest_y, sequence_num=0, verbose=False):
    request = RoutingRequest()

    # define header
    request.header.timestamp_sec = cyber_time.Time.now().to_sec()
    request.header.module_name = "automation routing"
    request.header.sequence_num = sequence_num

    # define way points (start and end)
    start_waypoint = request.waypoint.add()
    start_waypoint.pose.x = init_x
    start_waypoint.pose.y = init_y

    end_waypoint = request.waypoint.add()
    end_waypoint.pose.x = dest_x
    end_waypoint.pose.y = dest_y

    if verbose:
        print(
            f'routing request from: {init_x}, {init_y} to: {dest_x}, {dest_y} sent!')

    if not cyber.is_shutdown():
        writer.write(request)
    else:
        raise CyberShutdown

    cyber.shutdown()


def request_routing(init_x, init_y, dest_x, dest_y, verbose=False):
    cyber.init()

    if not cyber.ok():
        print('cyber error')
        sys.exit(1)

    # create node and writer
    node = create_node()
    writer = create_writer(node)

    time.sleep(2.0)
    # process message
    process_message(writer, init_x, init_y, dest_x,
                    dest_y, sequence_num=0, verbose=False)

def request_park_routing(ego_routing):
    cyber.init()

    if not cyber.ok():
        print('cyber error')
        sys.exit(1)

    
    node = cyber.Node("mock_park_routing_requester")
    sequence_num = 0

    routing_request = RoutingRequest()

    routing_request.header.timestamp_sec = cyber_time.Time.now().to_sec()
    routing_request.header.module_name = 'routing_request'
    routing_request.header.sequence_num = sequence_num
    sequence_num = sequence_num + 1

    waypoint_num = len(ego_routing["waypoints"])
    for i in range(waypoint_num):
        point1 = routing_request.waypoint.add()
        point1.pose.x = ego_routing["waypoints"][i]["x"]
        point1.pose.y = ego_routing["waypoints"][i]["y"]

    
    routing_request.parking_info.parking_space_id = ego_routing["park_info"]["parking_space_id"]
    parking_point = routing_request.parking_info.parking_point.add()
    parking_point.pose.x = ego_routing["parking_info"]["parking_point"]["x"]
    parking_point.pose.y = ego_routing["parking_info"]["parking_point"]["y"]


    writer = node.create_writer('/apollo/routing_request', RoutingRequest)
    time.sleep(2.0)
    print("routing_request", routing_request)
    writer.write(routing_request)
    time.sleep(2.0)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '-f', '--filename',
        help='The config file path of the ego\'s routing',
        type=str
    )
    args = parser.parse_args()

    request_park_routing(args.filename)


if __name__ == '__main__':
    main()
