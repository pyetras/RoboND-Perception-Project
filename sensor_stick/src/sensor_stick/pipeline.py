import pcl
from pcl_helper import ros_to_pcl, pcl_to_ros, XYZ_to_XYZRGB, XYZRGB_to_XYZ, get_color_list, rgb_to_float
import sensor_msgs.point_cloud2 as pc2
import numpy as np


def denoise(cloud):
  ''' Removes outlier voxels. '''
  outlier_filter = cloud.make_statistical_outlier_filter()
  outlier_filter.set_mean_k(10)
  outlier_filter.set_std_dev_mul_thresh(1.5)
  return outlier_filter.filter()


def voxel(cloud, leaf_size=0.01):
  ''' Downsamples the cloud using a voxel grid filter. '''
  vox = cloud.make_voxel_grid_filter()
  vox.set_leaf_size(leaf_size, leaf_size, leaf_size)
  return vox.filter()


def passth(cloud, axis='z', amin=0.6, amax=1.1):
  ''' Performs passthrough filtering. '''
  passthrough = cloud.make_passthrough_filter()

  # Assign axis and range to the passthrough filter object.
  passthrough.set_filter_field_name(axis)
  passthrough.set_filter_limits(amin, amax)

  # Finally use the filter function to obtain the resultant point cloud.
  return passthrough.filter()


def plane_points(cloud):
  ''' Fits a plane model using RANSAC to detect table points in the cloud. '''
  seg = cloud.make_segmenter()

  # Set the model you wish to fit
  seg.set_model_type(pcl.SACMODEL_PLANE)
  seg.set_method_type(pcl.SAC_RANSAC)

  # Max distance for a point to be considered fitting the model
  # Experiment with different values for max_distance
  # for segmenting the table
  max_distance = 0.01
  seg.set_distance_threshold(max_distance)

  # Call the segment function to obtain set of inlier indices and model
  # coefficients.
  inliers, _ = seg.segment()
  return inliers


def cluster_ix(cloud):
  white_cloud = XYZRGB_to_XYZ(cloud)
  tree = white_cloud.make_kdtree()
  ec = white_cloud.make_EuclideanClusterExtraction()
  ec.set_ClusterTolerance(0.05)
  ec.set_MinClusterSize(50)
  ec.set_MaxClusterSize(5000)
  # Search the k-d tree for clusters
  ec.set_SearchMethod(tree)
  # Extract indices for each of the discovered clusters
  cluster_indices = ec.Extract()
  return cluster_indices


def color_clusters(cloud, cluster_indices):
  nclust = len(cluster_indices)
  colors = get_color_list(nclust)

  arr = np.vstack(
      [cloud.extract(clust).to_array() for clust in cluster_indices])
  point_colors = [
      color[2] << 16 | color[1] << 8 | color[0]
      for clust, color in zip(cluster_indices, colors)
      for _ in range(len(clust))
  ]
  arr[:, 3] = np.array(point_colors, dtype=np.float32)
  clusters = pcl.PointCloud_PointXYZRGB()
  clusters.from_array(arr)
  return clusters
