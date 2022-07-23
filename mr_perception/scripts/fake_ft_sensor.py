#!/usr/bin/env python

import rospy
import tf
import tf.transformations as tfs
from tf.broadcaster import TransformBroadcaster
from tf.listener import TransformListener
from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from visualization_msgs.msg import *
from geometry_msgs.msg import Point, Quaternion, Wrench, WrenchStamped, Transform
import numpy as np


marker_pose = Transform()


def transformCallback(msg, tf_pub):
    tf_pub.sendTransform((marker_pose.translation.x, marker_pose.translation.y, marker_pose.translation.z), 
                         (0,0,0,1), 
                         rospy.Time.now(), 
                         "fake_wrench_marker", 
                         "world")

def publisherCallback(msg, listener, pub, ref_frame):
    try:
        # Get the fake_force_pose transform from the listener
        listener.waitForTransform("/world", "/fake_wrench_marker", rospy.Time(0), rospy.Duration(10.0))
        (trans1, rot1) = listener.lookupTransform("/world", "/fake_wrench_marker", rospy.Time(0))
        (trans2, rot2) = listener.lookupTransform("/world", ref_frame, rospy.Time(0))
        (trans3, rot3) = listener.lookupTransform(ref_frame, "/world", rospy.Time(0))
        
        # Calculate the force of wrench sensor
        mat_trans1 = tfs.translation_matrix(trans1)
        mat_trans2 = tfs.translation_matrix(trans2)
        mat_rot3 = tfs.quaternion_matrix(rot3)
        mat_vector = mat_trans1 - mat_trans2
        # mat_vector = mat_trans2 - mat_trans1
        mat_force = np.dot(mat_rot3, mat_vector)
        force_at_wrench = tfs.translation_from_matrix(mat_vector)
        # Construct the fake wrench data
        fake_wrench = WrenchStamped()
        fake_wrench.header.stamp = rospy.Time.now()
        fake_wrench.header.frame_id = ref_frame
        fake_wrench.wrench.force.x = force_at_wrench[0]
        fake_wrench.wrench.force.y = force_at_wrench[1]
        fake_wrench.wrench.force.z = force_at_wrench[2]
        rot_wrench = (marker_pose.rotation.x, marker_pose.rotation.y, marker_pose.rotation.z, marker_pose.rotation.w)
        torque_at_wrench = tfs.euler_from_quaternion(rot_wrench)
        fake_wrench.wrench.torque.x = torque_at_wrench[0]
        fake_wrench.wrench.torque.y = torque_at_wrench[1]
        fake_wrench.wrench.torque.z = torque_at_wrench[2]
        
        # Publish the fake wrench data
        pub.publish(fake_wrench)
        
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        print("Couldn't find transforms!")

def makeMarker(msg):
    # Construct a marker for controller
    marker = Marker()
    marker.type = Marker.CUBE
    marker.scale.x = msg.scale * 0.4
    marker.scale.y = msg.scale * 0.4
    marker.scale.z = msg.scale * 0.4
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.color.a = 1.0
    
    return marker

def controlMarker(msg):
    # Control the marker
    control = InteractiveMarkerControl()
    control.always_visible = True
    control.markers.append(makeMarker(msg))
    msg.controls.append(control)

def processFeedback(feedback, server):
    s = "Feedback from marker '" + feedback.marker_name
    s += "' / control '" + feedback.control_name + "'"
    
    mp = ""
    if feedback.mouse_point_valid:
        mp = " at " + str(feedback.mouse_point.x)
        mp += ", " + str(feedback.mouse_point.y)
        mp += ", " + str(feedback.mouse_point.z)
        mp += " in frame " + feedback.header.frame_id
    
    if feedback.event_type == InteractiveMarkerFeedback.BUTTON_CLICK:
        rospy.loginfo(s + ": button click" + mp + ".")
        pass
    elif feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
        rospy.loginfo(s + ": menu item " + str(feedback.menu_entry_id) + " clicked" + mp + ".")
        pass
    elif feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
        rospy.loginfo(s + ": pose updated" + mp + ".")
        marker_pose.translation.x = feedback.pose.position.x
        marker_pose.translation.y = feedback.pose.position.y
        marker_pose.translation.z = feedback.pose.position.z
        marker_pose.rotation.x = feedback.pose.orientation.x
        marker_pose.rotation.y = feedback.pose.orientation.y
        marker_pose.rotation.z = feedback.pose.orientation.z
        marker_pose.rotation.w = feedback.pose.orientation.w
    elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_DOWN:
        rospy.loginfo(s + ": mouse down" + mp + ".")
        pass
    elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
        rospy.loginfo(s + ": mouse up" + mp + ".")
        pass
    
    server.applyChanges()

if __name__ == "__main__":
    # Initialize the ros node
    rospy.init_node("fake_ft_sensor", anonymous=False)
    tf_broadcaster = TransformBroadcaster()
    tf_listener = TransformListener()
    pub_fake_wrench = rospy.Publisher('/fake_wrench', geometry_msgs.msg.WrenchStamped, queue_size=10)
    
    # Set the initial translation and rotation for marker_pose
    marker_pose.translation.x = 0.0
    marker_pose.translation.y = 0.0
    marker_pose.translation.z = 0.0
    marker_pose.rotation.x = 0.0
    marker_pose.rotation.y = 0.0
    marker_pose.rotation.z = 0.0
    marker_pose.rotation.w = 1.0
    
    # Publisher for fake_force_pose TF
    rospy.Timer(rospy.Duration(0.001), lambda msg: transformCallback(msg, tf_pub=tf_broadcaster))
    # Publisher for fake_wrench topic
    rospy.Timer(rospy.Duration(0.001), lambda msg: publisherCallback(msg, listener=tf_listener, pub=pub_fake_wrench, ref_frame="ee_link"))
    
    # Create an interactive marker server on the topic namespace simple_marker
    im_server = InteractiveMarkerServer("fake_ft_sensor")
    menu_handler = MenuHandler()
    pf_wrap = lambda fb: processFeedback(fb, im_server)
    menu_handler.insert("First Entry", callback=pf_wrap)
    menu_handler.insert("Second Entry", callback=pf_wrap)
    sub_menu_handler = menu_handler.insert("Submenu")
    menu_handler.insert("First Entry", parent=sub_menu_handler, callback=pf_wrap)
    menu_handler.insert("Second Entry", parent=sub_menu_handler, callback=pf_wrap)
    
    # Create an interactive marker for wrench sensor
    translation = Point(0.0, 0.0, 0.0)  # Set the initial position of interactive marker
    rotation = Quaternion(0.0, 0.0, 0.0, 1.0)  # Set the initial orientation of interactive marker
    wrench_marker = InteractiveMarker()
    wrench_marker.name = "fake_wrench_marker"
    wrench_marker.description = "Sim Wrench\nForce = vector from mr ee_link to fake_ft_marker\nTorque = angle from initial pose to fake_force_pose"
    wrench_marker.header.frame_id = "palm_link"
    wrench_marker.pose.position = translation
    wrench_marker.pose.orientation = rotation
    wrench_marker.scale = 0.2
    
    # Add a controller to the wrench_marker
    controlMarker(wrench_marker)
    
    # Control the wrench_marker
    fixed = False
    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 1
    control.orientation.y = 0
    control.orientation.z = 0
    control.name = "rotate_x"
    control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    if fixed:
        control.orientation_mode = InteractiveMarkerControl.FIXED
    wrench_marker.controls.append(control)
    
    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 1
    control.orientation.y = 0
    control.orientation.z = 0
    control.name = "move_x"
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    if fixed:
        control.orientation_mode = InteractiveMarkerControl.FIXED
    wrench_marker.controls.append(control)
    
    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 1
    control.orientation.z = 0
    control.name = "rotate_y"
    control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    if fixed:
        control.orientation_mode = InteractiveMarkerControl.FIXED
    wrench_marker.controls.append(control)
    
    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 1
    control.orientation.z = 0
    control.name = "move_y"
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    if fixed:
        control.orientation_mode = InteractiveMarkerControl.FIXED
    wrench_marker.controls.append(control)
    
    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 0
    control.orientation.z = 1
    control.name = "rotate_z"
    control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    if fixed:
        control.orientation_mode = InteractiveMarkerControl.FIXED
    wrench_marker.controls.append(control)
    
    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 0
    control.orientation.z = 1
    control.name = "move_z"
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    if fixed:
        control.orientation_mode = InteractiveMarkerControl.FIXED
    wrench_marker.controls.append(control)
    
    # Insert and apply the changes to the interactive marker server
    im_server.insert(wrench_marker, pf_wrap)
    menu_handler.apply(im_server, wrench_marker.name)
    im_server.applyChanges()
    
    rospy.spin()
