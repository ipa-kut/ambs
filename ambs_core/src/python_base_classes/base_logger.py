#!/usr/bin/env python
import rospy
import subprocess, shlex
from ambs_msgs.msg import BoolStamped, Float64Stamped
import threading
import sys
import os

##@ @package BaseLogger
#
# Provides base class for other loggers, mainly the rosbagger
#
class BaseLogger(object):
    def __init__(self, sub_dict = {}, result_list = [], param_list = [], rate = 100):
        self._rate = rospy.Rate(rate)

        self._input_interface = {
            "start"           : "in_start",
            "stop"            : "in_stop",
            "reset"           : "in_reset"
        }
        self._input_interface.update(sub_dict)
        self._config_param = []
        self._config_param += param_list

        # get ros param:
        self.config_param = {}
        self.getConfig(self._config_param)
        
        self._locks = {}
        self._flag = {}

        for key in self._input_interface:
            rospy.Subscriber(self._input_interface[key], BoolStamped, self.callback, key)
            self._locks[key] = threading.Lock()
            self._flag[key] = BoolStamped()
            self._flag[key].data = False
        
        self.result_list = result_list
        for topic in self.result_list:
            rospy.Subscriber(topic, Float64Stamped, self.callback, topic)
            self._locks[topic] = threading.Lock()
            self._flag[topic] = Float64Stamped()
            self._flag[topic].data = float("inf")

    def resetPorts(self):
        for key in self._input_interface:
            self._flag[key].data = False

    def setSafeFlag(self, key, value):
        if not key in self._input_interface.keys() and not key in self.result_list:
            rospy.logerr(rospy.get_name() + ": Retrieving a key that does not exist!: {}".format(key))
            return
        with self._locks[key]:
            self._flag[key] = value
    
    def callback(self, msg, key):
        self.setSafeFlag(key,msg)

    def getSafeFlag(self, key):
        if not key in self._input_interface.keys() and not key in self.result_list:
            rospy.logerr(rospy.get_name() + ": Retrieving a key that does not exist!: {}".format(key))
            return
        else:
            with self._locks[key]:
                return self._flag[key].data

    def startCommandProc(self, command):
        # Prepare command
        _command = shlex.split(command)
        self.command_proc = subprocess.Popen(_command)
        rospy.loginfo(rospy.get_name() + ": Starting process {}".format(self.command_proc))

    def killCommandProc(self):
        self.command_proc.send_signal(subprocess.signal.SIGINT)
        rospy.loginfo(rospy.get_name() + ": Killing process {}".format(self.command_proc))
    
    def getConfig(self, param_list):
        for arg in param_list:
            if rospy.has_param(arg):
                if arg not in self.config_param:
                    self.config_param[arg] = rospy.get_param(arg)
            else:
                rospy.logerr("{} param not set!!".format(arg))
                rospy.signal_shutdown("param not set")

    def logScreenFile(self, log_msg):
        if self.config_param[rospy.get_name() + "/screen"]:
            rospy.logwarn(rospy.get_name() + ": " + log_msg)
            rospy.loginfo(rospy.get_name() + ": " + log_msg)
        else:
            rospy.loginfo(rospy.get_name() + ": " + log_msg)
