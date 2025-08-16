#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Python 2.7 (ROS Melodic)

import rospy
import subprocess
import os

from viohar_prediction.srv import StartPipeline, StartPipelineResponse
from viohar_prediction.srv import StopPipeline, StopPipelineResponse

# global variable to hold the roslaunch subprocess
launch_parent_process = None

def start_pipeline(req):
    global launch_parent_process

    if launch_parent_process is not None:
        return StartPipelineResponse(False, "Pipeline already running. Click Stop first.")

    bag_path = req.bag_file.strip()

    if not bag_path:
        return StartPipelineResponse(False, "Empty bag_file.")
    if not os.path.isabs(bag_path):
        return StartPipelineResponse(False, "Please provide an absolute path to the .bag file.")
    if not os.path.exists(bag_path):
        return StartPipelineResponse(False, "Bag file not found: {}".format(bag_path))

    # Build the roslaunch command
    launch_cmd = [
        "roslaunch",
        "vins_auto_launcher",
        "full_pipeline_predict.launch",
        "bag_file:={}".format(bag_path)
    ]

    rospy.loginfo("Starting pipeline with command: %s", ' '.join(launch_cmd))

    try:
        # Start as a subprocess
        launch_parent_process = subprocess.Popen(launch_cmd)
        rospy.loginfo("Pipeline started successfully.")
        return StartPipelineResponse(True, "Pipeline started.")
    except Exception as e:
        rospy.logerr("Failed to start pipeline: %s", e)
        launch_parent_process = None
        return StartPipelineResponse(False, "Launch error: {}".format(e))

def stop_pipeline(req):
    global launch_parent_process

    if launch_parent_process is None:
        return StopPipelineResponse(False, "No pipeline running.")

    try:
        rospy.loginfo("Stopping pipeline...")
        launch_parent_process.terminate()  # graceful stop
        launch_parent_process.wait()       # wait for process to exit
        launch_parent_process = None
        rospy.loginfo("Pipeline stopped.")
        return StopPipelineResponse(True, "Pipeline stopped.")
    except Exception as e:
        rospy.logerr("Failed to stop pipeline: %s", e)
        return StopPipelineResponse(False, "Stop error: {}".format(e))

def main():
    rospy.init_node('viohar_pipeline_launcher', anonymous=False)

    # advertise services
    s_start = rospy.Service('/viohar/start', StartPipeline, start_pipeline)
    s_stop  = rospy.Service('/viohar/stop', StopPipeline, stop_pipeline)

    rospy.loginfo("viohar_pipeline_launcher ready: services /viohar/start and /viohar/stop.")
    rospy.spin()

if __name__ == '__main__':
    main()
