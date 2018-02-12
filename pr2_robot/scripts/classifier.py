#!/usr/bin/env python

import numpy as np
import pcl
import pickle
import rospy
import sensor_msgs.point_cloud2 as pc2
from sklearn.preprocessing import LabelEncoder
from visualization_msgs.msg import Marker

from sensor_stick.marker_tools import make_label
from sensor_stick.msg import DetectedObjectsArray
from sensor_stick.msg import DetectedObject
from sensor_stick import features
from sensor_stick import pipeline
from sensor_stick.pcl_helper import ros_to_pcl, pcl_to_ros
from sensor_stick.srv import GetNormals
from rospy_message_converter import message_converter
import yaml


def get_normals(cloud):
  get_normals_prox = rospy.ServiceProxy('/feature_extractor/get_normals',
                                        GetNormals)
  return get_normals_prox(cloud).cluster


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


# Callback function for your Point Cloud Subscriber
def pcl_callback(pcl_msg):
  cloud = ros_to_pcl(pcl_msg)
  cloud = pipeline.passth(cloud)
  cloud = pipeline.passth(cloud, axis='y', amin=-0.45, amax=0.45)
  cloud = pipeline.denoise(cloud)
  filtered = pipeline.voxel(cloud)

  table_points = pipeline.plane_points(filtered)
  table_cloud = filtered.extract(table_points, negative=False)
  obj_cloud = filtered.extract(table_points, negative=True)

  pcl_objects_pub.publish(pcl_to_ros(obj_cloud))
  pcl_table_pub.publish(pcl_to_ros(table_cloud))

  cluster_ix = pipeline.cluster_ix(obj_cloud)

  pcl_cluster_pub.publish(
      pcl_to_ros(pipeline.color_clusters(obj_cloud, cluster_ix)))

  detected_objects = []

  for i, ixs in enumerate(cluster_ix):
    pcl_data = obj_cloud.extract(ixs)
    ros_data = pcl_to_ros(pcl_data)
    feature = np.concatenate((features.compute_color_histograms(ros_data),))
    prediction = clf.predict(scaler.transform(feature.reshape(1, -1)))
    label = encoder.inverse_transform(prediction)[0]

    label_pos = list(next(pc2.read_points(ros_data))[:3])
    label_pos[2] += .4
    object_markers_pub.publish(make_label(label, label_pos, i))

    do = DetectedObject()
    do.label = label
    do.cloud = ros_data
    detected_objects.append(do)

  detected_objects_pub.publish(detected_objects)


if __name__ == '__main__':
  rospy.init_node('classifier', anonymous=True)
  pcl_sub = rospy.Subscriber(
      '/pr2/world/points', pc2.PointCloud2, pcl_callback, queue_size=1)
  pcl_objects_pub = rospy.Publisher(
      "/pcl_objects", pc2.PointCloud2, queue_size=1)
  pcl_cluster_pub = rospy.Publisher(
      "/pcl_cluster", pc2.PointCloud2, queue_size=1)
  pcl_table_pub = rospy.Publisher("/pcl_table", pc2.PointCloud2, queue_size=1)

  object_markers_pub = rospy.Publisher('/object_markers', Marker, queue_size=1)
  detected_objects_pub = rospy.Publisher(
      '/detected_objects', DetectedObjectsArray, queue_size=1)

  model = pickle.load(open('/home/robond/catkin_ws/src/model.sav', 'r'))
  clf = model['classifier']
  encoder = LabelEncoder()
  encoder.classes_ = model['classes']
  scaler = model['scaler']

  while not rospy.is_shutdown():
    rospy.spin()
