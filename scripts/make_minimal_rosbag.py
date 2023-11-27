#!/usr/bin/env python

import argparse

import progressbar

import rosbag

topic_whitelist = ['/tf', '/tf_static', '/bus/vls128_roof/eth_scan/bus_to_host', '/bus/os32_left/lidar_packets',
                   '/bus/os32_right/lidar_packets']

parser = argparse.ArgumentParser(description='Creates minimal rosbag')
parser.add_argument('--input_bag', type=str, help='specify input bag', required=True)
parser.add_argument('--output_bag', type=str, help='specify output bag', required=True)
parser.add_argument('--start_time', type=str, help='specify start time relative to bag start', default=0)
parser.add_argument('--end_time', type=str, help='specify end time relative to bag start', default=-1)
args = parser.parse_args()

start = float(args.start_time)
end = float(args.end_time)

record_stamp_of_first_msg = None
latched_messages_before_start = []

with rosbag.Bag(args.output_bag, 'w') as output_bag:
    input_bag = rosbag.Bag(args.input_bag)
    bar = progressbar.ProgressBar(input_bag.get_message_count())
    for m in bar(input_bag.read_messages(return_connection_header=True)):

        # calculate the relative time delta between current and first message
        if record_stamp_of_first_msg is None:
            record_stamp_of_first_msg = m.timestamp
        t_rel = (m.timestamp - record_stamp_of_first_msg).to_sec()

        # check if topic is in whitelist
        if m.topic in topic_whitelist:
            if t_rel < start and 'latching' in m.connection_header and m.connection_header['latching'] == b'1':
                # latched message before start; save and insert it later
                latched_messages_before_start.append(m)
            elif t_rel >= start and (t_rel < end or end < 0):
                # before we add the first message we insert all latched messages
                while len(latched_messages_before_start) > 0:
                    m_latched = latched_messages_before_start.pop(0)
                    output_bag.write(m_latched.topic, m_latched.message, m.timestamp,
                                     connection_header=m_latched.connection_header)  # using timestamp of m to avoid gap

                # write message
                output_bag.write(m.topic, m.message, m.timestamp, connection_header=m.connection_header)
