#!/usr/bin/env python

import rospy
import subprocess, shlex
from ambs_msgs.msg import BoolStamped
from python_base_classes.base_logger import BaseLogger
import threading
from datetime import datetime
from std_msgs.msg import String
import os

class Rosbagger(BaseLogger):
    def __init__(self):

        self.node_name = rospy.get_name()

        extend_subscribers_dict = []

        param_list = [
            self.node_name + "/blacklist",
            self.node_name + "/whitelist"
            ]

        super(Rosbagger, self).__init__(sub_dict= extend_subscribers_dict, param_list = param_list)

        # Default whitelist topics that should always be recorded for every test. This is a pattern search.
        self.whitelist = ["ambs"]
        # Optional whitelist topics that are test specific
        self.whitelist = self.whitelist + self.config_param[self.node_name + "/whitelist"]

        self.begin_write_pub = rospy.Publisher('out_rosbag_began', BoolStamped, queue_size=10, latch=True)

        try:
            while not rospy.is_shutdown():
                self.main_loop() 
        except rospy.ROSException:
            pass 

    def buildNewBoolStamped(self, data = True):
        msg = BoolStamped()
        msg.header.stamp = rospy.Time.now()
        msg.data = data
        return msg

    def rosbag_begin_cb(self, msg):
        if not self._published_once:
            self.begin_write_pub.publish(self.buildNewBoolStamped(True))
            self._published_once = True

    def prepare_topics(self):
        list_of_topics = rospy.get_published_topics()
        list_of_topics.sort()
        topics_string = ""

        for topic,msg in list_of_topics:
            topic_allowed = False
            for white_string in self.whitelist:
                if white_string in topic:
                    topic_allowed = True
                    break
            for black_string in self.config_param[self.node_name + "/blacklist"]:
                if black_string in topic:
                    topic_allowed = False
                    break
            if topic_allowed:
                 topics_string = topics_string + topic + " "

        return topics_string

    def main_loop(self):
        # Setup a subscriber to listen to when recording begins
        # Subscriber publishes a signal when recording actually begins
        # Set initial signal to false
        self._published_once = False
        self.begin_write_pub.publish(self.buildNewBoolStamped(False))
        rospy.Subscriber("begin_write", String, self.rosbag_begin_cb)

        rospy.loginfo(rospy.get_name() + ": Waiting for start signal...")
        # Wait for start signal
        start = False
        stop = False
        reset = False

        while not start and not rospy.is_shutdown():
            self._rate.sleep()
            start = self.getSafeFlag("start")

         # Create folder -> sleep -> start recording
        if start == True and stop == False:
            rospy.loginfo(rospy.get_name() + ": Start received")

            log_folder = ""
            while not rospy.has_param("log_folder"):
                print("Waiting for log_folder")
            log_folder = rospy.get_param("log_folder")

            topics_string = self.prepare_topics()
            command = "rosbag record -p -o " + log_folder + "/ " + topics_string
            self.startCommandProc(command)

        # Wait for stop signal
        while not stop and not rospy.is_shutdown():
            self._rate.sleep()
            stop = self.getSafeFlag("stop")
        
        # Kill recorder
        if start == True:
            if stop or rospy.is_shutdown():
                rospy.loginfo(rospy.get_name() + ": Stop received")
                self.killCommandProc()

        rospy.loginfo(rospy.get_name() + ": Waiting for reset signal...")
        # Wait for reset
        while not reset and not rospy.is_shutdown():
            self._rate.sleep()
            reset = self.getSafeFlag("reset")
        
