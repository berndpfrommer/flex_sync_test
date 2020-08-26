#!/usr/bin/python
# 2020 converts message type from sensor_msgs:
#
# Image -> Illuminance
# CameraInfo -> Temperature
#
# This reduces the size of the bag
#
# usage: python convert_bag_type.py -i <path_to_input_bagfile> -o <path_to_output_bagfile>

import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Illuminance
from sensor_msgs.msg import Temperature

import rosbag
import sys
import argparse

def parse_arguments():
    parser = argparse.ArgumentParser()
    parser.add_argument('-i', '--input_bag', help='input bag name',
                        required=True, default=None)
    parser.add_argument('-o', '--output_bag', help='output bag name',
                        required=True, default=None)
    return parser.parse_args()

if  __name__ == '__main__':
    args = parse_arguments()
    print 'converting bag {:s} -> {:s}'.format(args.input_bag, args.output_bag)

    illuminance_topics = [
        '/t265/fisheye1/image_raw',
        '/right_tof/stream/1/mono8',
        '/left_tof/stream/1/mono8']
    temperature_topics = [
        '/left_tof/depth/camera_info',
        '/right_tof/depth/camera_info',
        '/t265/fisheye1/camera_info']
    
    num_converted = 0
    with rosbag.Bag(args.input_bag, mode='r') as in_bag:
        with rosbag.Bag(args.output_bag, mode='w') as out_bag:
            iterator = in_bag.read_messages()
            for (topic, msg, t) in iterator:
                if topic in illuminance_topics:
                    m = Illuminance(header=msg.header)
                    out_bag.write(topic, m, t)
                    num_converted += 1
                elif topic in temperature_topics:
                    m = Temperature(header=msg.header)
                    out_bag.write(topic, m, t)
                    num_converted += 1
            out_bag.close()
        in_bag.close()
    print 'converted {:d} messages!'.format(num_converted)
    
