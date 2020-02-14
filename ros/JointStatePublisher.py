#!/usr/bin/env python

import roslibpy
import math, time

class JointStatePublisher(object):
    def __init__(self):
        self.ros = roslibpy.Ros(host='localhost', port=9090)
        self.ros.on_ready(lambda: print('ROS Connection:', self.ros.is_connected))
        self.pub = roslibpy.Topic(self.ros, 'joint_states', 'sensor_msgs/JointState')
        try:
            self.ros.run()
        except KeyboardInterrupt:
            self.ros.terminate()

    def publish(self, joint_names, joint_values_grbl):
        joint_values_ros = self.convertJointsRosFormat(joint_values_grbl)
        self.pub.publish(roslibpy.Message({'header':{'stamp': time.time() }, 'name': joint_names, 'position': joint_values_ros}))

    def convertJointsRosFormat(self, joint_values_in_mm_degree):
        joint_values_in_m_radius = [joint_values_in_mm_degree[0]/1000.0, joint_values_in_mm_degree[1]/1000.0, joint_values_in_mm_degree[2]/1000.0, joint_values_in_mm_degree[3] * math.pi / 180, joint_values_in_mm_degree[4] * math.pi / 180]
        return joint_values_in_m_radius