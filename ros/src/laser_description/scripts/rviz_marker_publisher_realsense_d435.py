#!/usr/bin/env python

import roslib
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
import rospy
import math

publisher = rospy.Publisher('/camera/rviz_marker', Marker, queue_size=10)

rospy.init_node('rviz_marker_realsense_camera')

while not rospy.is_shutdown():
    marker = Marker()
    marker.header.frame_id = "camera_color_optical_frame"
    marker.ns = "camera"
    marker.id = 0
    marker.header.stamp = rospy.Time.now()
    marker.type = marker.MESH_RESOURCE
    marker.mesh_resource = "package://realsense2_description/meshes/d415.stl"
    marker.mesh_use_embedded_materials = True

    marker.pose.position = Point(0, 0, 0)
    marker.pose.orientation.x = 0
    marker.pose.orientation.y = 0
    marker.pose.orientation.z = 0
    marker.pose.orientation.w = 1.0
    # marker.scale.x = 0.001
    # marker.scale.y = 0.001
    # marker.scale.z = 0.001
    # marker.lifetime = rospy.Duration()
    # marker.frame_locked = True

    publisher.publish(marker)

    rospy.sleep(0.01)