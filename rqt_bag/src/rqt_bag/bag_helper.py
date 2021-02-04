# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICTS
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""
Helper functions for bag files and timestamps.
"""
from __future__ import print_function
import functools
import os
import math
# import importlib
import time
import rospy
# import rosbag
import rospkg
import numpy as np
import subprocess, yaml

rp = rospkg.RosPack()
msg_map_file = os.path.join(rp.get_path('rqt_bag'), 'config', 'msg_map.yaml')
with open(msg_map_file, 'r') as f:
    msg_map = yaml.load(f)

SLIDER_BAR_MAX = 100000

def timeit(func):
    @functools.wraps(func)
    def wrapper(*args, **kargs):
        start = time.time()
        result = func(*args, **kargs)
        stop = time.time()
        print(stop - start)
        return result
    return wrapper

def notNoneAttrs(*notNoneAttrs):
    def decorator(func):
        @functools.wraps(func)
        def wrapper(self, *args, **kwargs):
            for notNoneAttr in notNoneAttrs:
                if getattr(self, notNoneAttr) == None:
                    return None
            return func(self, *args, **kwargs)
        return wrapper
    return decorator


def stamp_to_str(t):
    """
    Convert a rospy.Time to a human-readable string.

    @param t: time to convert
    @type  t: rospy.Time
    """
    if isinstance(t, float):
        t_sec = t
        t = rospy.Time.from_sec(t)
    else:
        t_sec = t.to_sec()
    if t < rospy.Time.from_sec(60 * 60 * 24 * 365 * 5):
        # Display timestamps earlier than 1975 as seconds
        return '%.3fs' % t_sec
    else:
        return time.strftime('%b %d %Y %H:%M:%S', time.localtime(t_sec)) + '.%03d' % (t.nsecs / 1000000)


def get_topics(bag):
    """
    Get an alphabetical list of all the unique topics in the bag.

    @return: sorted list of topics
    @rtype:  list of str
    """
    return sorted(set([c.topic for c in bag._get_connections()]))


def get_start_stamp(bag):
    """
    Get the earliest timestamp in the bag.

    @param bag: bag file
    @type  bag: rosbag.Bag
    @return: earliest timestamp
    @rtype:  rospy.Time
    """
    start_stamp = None
    for connection_start_stamp in [index[0].time for index in bag._connection_indexes.values()]:
        if not start_stamp or connection_start_stamp < start_stamp:
            start_stamp = connection_start_stamp
    return start_stamp


def get_end_stamp(bag):
    """
    Get the latest timestamp in the bag.

    @param bag: bag file
    @type  bag: rosbag.Bag
    @return: latest timestamp
    @rtype:  rospy.Time
    """
    end_stamp = None
    for connection_end_stamp in [index[-1].time for index in bag._connection_indexes.values()]:
        if not end_stamp or connection_end_stamp > end_stamp:
            end_stamp = connection_end_stamp

    return end_stamp


def get_topics_by_datatype(bag):
    """
    Get all the message types in the bag and their associated topics.

    @param bag: bag file
    @type  bag: rosbag.Bag
    @return: mapping from message typename to list of topics
    @rtype:  dict of str to list of str
    """
    topics_by_datatype = {}
    for c in bag._get_connections():
        topics_by_datatype.setdefault(c.datatype, []).append(c.topic)

    return topics_by_datatype

# Get lists of topics and types from a bag
def get_topics_and_types(bag):
    topics = bag.get_type_and_topic_info()[1].keys()
    types = []
    for i in range(0,len(bag.get_type_and_topic_info()[1].values())):
        types.append(bag.get_type_and_topic_info()[1].values()[i][0])
    return topics, types


def get_datatype(bag, topic):
    """
    Get the datatype of the given topic.

    @param bag: bag file
    @type  bag: rosbag.Bag
    @return: message typename
    @rtype:  str
    """
    for c in bag._get_connections(topic):
        return c.datatype

    return None


def filesize_to_str(size):
    size_name = ('B', 'KB', 'MB', 'GB', 'TB', 'PB', 'EB', 'ZB', 'YB')
    i = int(math.floor(math.log(size, 1024)))
    p = math.pow(1024, i)
    s = round(size / p, 2)
    if s > 0:
        return '%s %s' % (s, size_name[i])
    return '0 B'

# Rewrite bag with header timestamps
# This is useful in the case that the message receipt time substantially differs from the generation time, 
# e.g. when messages are recorded over an unreliable or slow connection.

# Note that this could potentially change the order in which messages are republished by rosbag play.

def rewrite_ts_header(outbag, inputbag):
    for topic, msg, t in inputbag.read_messages():
        # This also replaces tf timestamps under the assumption 
        # that all transforms in the message share the same timestamp
        if topic == "/tf" and msg.transforms:
            outbag.write(topic, msg, msg.transforms[0].header.stamp)
        else:
            outbag.write(topic, msg, msg.header.stamp if msg._has_header else t)


# Add metadata to a bag
def add_metadata(bag, metadata_msg):
    bag.write('/metadata', metadata_msg, rospy.Time(bag.get_end_time()))


# Get summary information about a bag
def get_summary_info_exec(bag_file):
    info_dict = yaml.load(subprocess.Popen(['rosbag', 'info', '--yaml', bag_file], stdout=subprocess.PIPE).communicate()[0])
    return info_dict


def get_summary_info(bag):
    info_dict = yaml.load(bag._get_yaml_info())


# Create a cropped bagfile
def gen_cropped_bag(outbag, inputbag, num_msgs):
    for topic, msg, t in inputbag.read_messages():
        while num_msgs:
            outbag.write(topic, msg, t)
            num_msgs -= 1

# def get_pytype_by_datatype(datatype):
#     try:
#         the_msg_map = msg_map[datatype]
#         return getattr(importlib.import_module(the_msg_map['module']), the_msg_map['class'])
#     except Exception:
#         return None
