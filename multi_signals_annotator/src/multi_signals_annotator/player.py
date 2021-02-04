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
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""
Player listens to messages from the timeline and publishes them to ROS.
"""
import os
import signal

import rospy
import rosgraph_msgs

import subprocess

from python_qt_binding.QtCore import QObject, QProcess, Signal

CLOCK_TOPIC = "/clock"


class Player(QObject):

    errorSignal = Signal(str) 
    outputSignal = Signal(str)

    cmd_rosbag = 'rosbag play --try-future-version'
    cmd_delay = ' -d '
    cmd_duration = ' -u '
    cmd_start = ' -s '
    cmd_publish_clock = ' --clock '
    cmd_multiply_factor = ' -r '
    cmd_select_topics = ' --topics '
    cmd_bag_file = ' --bags='


    def __init__(self, timeline):
        super(Player, self).__init__()
        self.timeline = timeline
        self.is_publishing = False
        self._cmd_base = self.cmd_rosbag + self.cmd_delay + '0'

        self._process = QProcess()
        self._process.readyReadStandardError.connect(self._onReadyReadStandardError)
        self._process.readyReadStandardOutput.connect(self._onReadyReadStandardOutput)

        self._playhead_offset = 0.0

    def _onReadyReadStandardError(self):
        result = self._process.readAllStandardError().data().decode()
        self.errorSignal.emit(result)

    def _onReadyReadStandardOutput(self):
        result = self._process.readAllStandardOutput().data().decode()
        self.outputSignal.emit(result)

    def _system_call(self, command):
        """
        Executes a system command.
        """
        self._process.waitForFinished(30000)
        self._process.start(command)

    def start_publishing(self, playhead_offset=0.0, speed=1.0):
        if self.is_publishing:
            self.stop_publishing()

        cmd = self._cmd_base + self.cmd_bag_file + '\"' + self.timeline.bag_filename + '\"'
        if not isinstance(playhead_offset, float):
            playhead_offset = 0.0
        cmd = cmd + self.cmd_start + str(playhead_offset)
        self._playhead_offset = playhead_offset
        if not isinstance(speed, float):
            speed = 1.0
        cmd = cmd + self.cmd_multiply_factor + str(speed)
        self._system_call(cmd)
        self.is_publishing = True

    def stop_publishing(self):
        if self._process.processId() != 0:
            self.terminate_process_and_children()
        self.is_publishing = False
        

    def terminate_process_and_children(self):
        ps_command = subprocess.Popen("ps -o pid --ppid %d --noheaders" % self._process.processId(), shell=True, stdout=subprocess.PIPE)
        ps_output = ps_command.stdout.read()
        retcode = ps_command.wait()
        assert retcode == 0, "ps command returned %d" % retcode
        for pid_str in ps_output.split("\n")[:-1]:
            os.kill(int(pid_str), signal.SIGINT)
        os.kill(self._process.processId(), signal.SIGINT)

    def stop(self):
        self.stop_publishing()