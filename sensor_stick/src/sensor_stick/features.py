import matplotlib.colors
import matplotlib.pyplot as plt
import numpy as np
import pcl_helper
import sensor_msgs.point_cloud2 as pc2


def rgb_to_hsv(rgb_list):
  rgb_normalized = [
      1.0 * rgb_list[0] / 255, 1.0 * rgb_list[1] / 255, 1.0 * rgb_list[2] / 255
  ]
  hsv_normalized = matplotlib.colors.rgb_to_hsv([[rgb_normalized]])[0][0]
  return hsv_normalized


def compute_color_histograms(cloud, using_hsv=True):
  point_colors_list = []

  # Step through each point in the point cloud
  for point in pc2.read_points(cloud, skip_nans=True):
    rgb_list = pcl_helper.float_to_rgb(point[3])
    if using_hsv:
      point_colors_list.append(rgb_to_hsv(rgb_list) * 255)
    else:
      point_colors_list.append(rgb_list)

  # Populate lists with color values
  colors = np.array(point_colors_list)
  hists = np.apply_along_axis(
      np.histogram, 0, colors, bins=32, range=(0, 256))[0]
  features = np.hstack(hists).astype(np.float64)
  return features / np.sum(features)


def compute_normal_histograms(normal_cloud):
  norms = np.array(
      list(
          pc2.read_points(
              normal_cloud,
              skip_nans=True,
              field_names=('normal_x', 'normal_y', 'normal_z'))))
  hists = np.apply_along_axis(
      np.histogram, 0, norms, bins=32, range=(-1., 1.))[0]
  feats = np.concatenate(hists).astype(np.float64)
  return feats / np.sum(feats)
