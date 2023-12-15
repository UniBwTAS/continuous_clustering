#!/usr/bin/env python

import argparse

import cv2 as cv
import numpy as np
import progressbar
from cv_bridge import CvBridge
from image_geometry import PinholeCameraModel

import rosbag


def generate_rectify_map(cam_model):
    pixel_coordinate_mapping = np.ndarray(shape=(cam_model.height, cam_model.width, 2), dtype="float32")
    cv.initUndistortRectifyMap(cam_model.full_K, cam_model.D, cam_model.R, cam_model.full_P,
                               (cam_model.width, cam_model.height), cv.CV_32FC2, pixel_coordinate_mapping, None)

    # account for region of interest (roi)
    pixel_coordinate_mapping = pixel_coordinate_mapping[
                               cam_model.raw_roi.y_offset:cam_model.raw_roi.y_offset + cam_model.raw_roi.height,
                               cam_model.raw_roi.x_offset:cam_model.raw_roi.x_offset + cam_model.raw_roi.width]
    pixel_coordinate_mapping -= np.array([cam_model.raw_roi.x_offset, cam_model.raw_roi.y_offset])
    return pixel_coordinate_mapping


def debayer_blur_and_rectify_image(msg_image, topic):
    cv_image = open_cv_bridge.imgmsg_to_cv2(msg_image, desired_encoding='rgb8')
    cv_image = cv.GaussianBlur(cv_image, (41, 41), 0)
    # try to rectify image if camera info is already available
    try:
        cv_image_rectified = cv.remap(cv_image, rectify_maps[topic], None, cv.INTER_CUBIC)
    except KeyError:
        cv_image_rectified = cv_image
    out = open_cv_bridge.cv2_to_imgmsg(cv_image_rectified, encoding="rgb8")
    out.header = msg_image.header
    return out


topic_whitelist = [
    '/tf',
    '/tf_static',
    '/bus/vls128_roof/eth_scan/bus_to_host',
    '/bus/os32_left/lidar_packets',
    '/bus/os32_right/lidar_packets',
    '/sensor/camera/surround/front/image_raw',
    '/sensor/camera/surround/front/camera_info'
]

# window_name = 'Remap demo'
# cv.namedWindow(window_name)

parser = argparse.ArgumentParser(description='Creates minimal rosbag')
parser.add_argument('--input_bag', type=str, help='specify input bag', required=True)
parser.add_argument('--output_bag', type=str, help='specify output bag', required=True)
parser.add_argument('--start_time', type=str, help='specify start time relative to bag start', default=0)
parser.add_argument('--end_time', type=str, help='specify end time relative to bag start', default=-1)
parser.add_argument('--blur_image', action='store_true')
args = parser.parse_args()

start = float(args.start_time)
end = float(args.end_time)

open_cv_bridge = CvBridge()
rectify_maps = dict()

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

            # generate rectify map from camera info in order to be able to rectify the raw image later
            if args.blur_image and m.connection_header['type'] == b'sensor_msgs/CameraInfo' and m.topic not in rectify_maps:
                model = PinholeCameraModel()
                model.fromCameraInfo(m.message)
                rectify_maps[m.topic.replace("/camera_info", "")] = generate_rectify_map(model)

            # check if image and whether it should be blurred
            if args.blur_image and m.connection_header['type'] == b'sensor_msgs/Image' and m.topic.endswith(
                    'image_raw'):
                msg = debayer_blur_and_rectify_image(m.message, m.topic.replace("/image_raw", ""))
                topic = m.topic[:-9] + "image_rect_color"
            else:
                msg = m.message
                topic = m.topic

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
                output_bag.write(topic, msg, m.timestamp, connection_header=m.connection_header)
