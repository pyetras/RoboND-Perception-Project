#!/usr/bin/env python

import numpy as np
import rospy
import yaml
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from std_msgs.msg import Int32

from sensor_stick.msg import DetectedObjectsArray
from rospy_message_converter import message_converter
from sensor_stick.pcl_helper import ros_to_pcl


# Helper function to create a yaml friendly dictionary from ROS messages
def make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose,
                   place_pose):
  yaml_dict = {}
  yaml_dict["test_scene_num"] = test_scene_num.data
  yaml_dict["arm_name"] = arm_name.data
  yaml_dict["object_name"] = object_name.data
  yaml_dict["pick_pose"] = message_converter.convert_ros_message_to_dictionary(
      pick_pose)
  yaml_dict["place_pose"] = message_converter.convert_ros_message_to_dictionary(
      place_pose)
  return yaml_dict


# Helper function to output to yaml file
def send_to_yaml(yaml_filename, dict_list):
  data_dict = {"object_list": dict_list}
  with open(yaml_filename, 'w') as outfile:
    yaml.dump(data_dict, outfile, default_flow_style=False)


def obj_callback(objs):
  centroids = {}
  for obj in objs.objects:
    centroids[obj.label] = map(np.asscalar,
                               np.mean(
                                   ros_to_pcl(obj.cloud).to_array()[:, :3],
                                   axis=0))

  output = []
  for config in object_list_param:
    if not config['name'] in centroids:
      continue
    i = Int32()
    i.data = 1
    name = String()
    name.data = config['name']
    arm = String()
    arm.data = dropbox[config['group']]['name']
    pick_pose = Pose()
    pick_pose.position.x, pick_pose.position.y, pick_pose.position.z = centroids[
        config['name']]
    place_pose = Pose()
    place_pose.position.x, place_pose.position.y, place_pose.position.z = dropbox[
        config['group']]['position']
    data = make_yaml_dict(i, arm, name, pick_pose, place_pose)
    output.append(data)

  print(output)
  send_to_yaml('output_1.yaml', output)


if __name__ == '__main__':
  rospy.init_node('instructions', anonymous=True)
  obj_sub = rospy.Subscriber(
      '/detected_objects', DetectedObjectsArray, obj_callback, queue_size=1)
  object_list_param = rospy.get_param('/object_list')
  dropbox_list = rospy.get_param('/dropbox')
  dropbox = dict([(x['group'], x) for x in dropbox_list])

  while not rospy.is_shutdown():
    rospy.spin()
