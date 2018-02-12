## Project: Perception Pick & Place
## [Rubric](https://review.udacity.com/#!/rubrics/1067/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.

[//]: # (Image References)

[noisy]: ./images/noisy.png
[filtered]: ./images/filtered.png
[table]: ./images/table.png
[clusters]: ./images/clusters.png
[conf1]: ./images/conf1.png
[conf2]: ./images/conf2.png
[world1]: ./images/world1.png
[world2]: ./images/world2.png
[world3]: ./images/world3.png

---
### Writeup / README

Object recognition has been implemented in `pr2_robot/scripts/classifier.py`.
I have implemented the instruction (YAML) output as another node in `pr2_robot/scripts/instruction.py`. Generated output files are in `data/`

Implementation of the cloud preprocessing pipeline is in `sensor_stick/src/sensor_stick/pipeline.py`.

### Exercise 1, 2 and 3 pipeline implemented
#### 1. Complete Exercise 1 steps. Pipeline for filtering and RANSAC plane fitting implemented.

Original point cloud:
![pcl][noisy]

The point cloud is run through a passthrough filter, eliminating points outside of [-0.45, 0.45] in the y dimension and [0.6, 1.1] in the z dimension.
Later the cloud is denoised using `statistical_outlier_filter(K = 10, STDDEV = 1.5)`.

![pcl][filtered]

Only then the data is downsampled using `voxel_grid_filter(LEAF_SIZE = 0.01)`.
Then a plane model is fitted using `RANSAC(MAX_DISTANCE = 0.01)` to detect table points.

![table][table]

#### 2. Complete Exercise 2 steps: Pipeline including clustering for segmentation implemented.

The remaining points are split into Euclidean clusters and processed separately in the next stage.
The color information is stripped from the data and `EuclideanClusterExtraction(TOLERANCE = 0.05, MINSIZE = 50, MAXSIZE = 5000)` is performed.
The clusters are additionally outputted with different colors for inspection.

![clusters][clusters]

#### 3. Complete Exercise 3 Steps.  Features extracted and SVM trained.  Object recognition implemented.

`compute_color_histograms()` and `compute_normal_histograms()` are implemented in `sensor_stick/src/sensor_stick/features.py`.
I have created 32 bins for each dimension of HSV color data and normals data collected the samples using `capture_features.py`.
I have experimented with downsampling the data with the voxel filter before extracting features and speeding up the collection process, so that I can collect more samples.
However, I've found that 500 downsampled features did not work as well as 50 features with full resolution, even though the accuracy score was slightly better on the training set.
In the end I used a model trained on the full resolution-low count data.
Furthermore, I've noticed that the normals histograms lower the performance of the classifier, so even though I collected them, I chose to remove them before training the model.

The model has been trained using the provided script, I have not made any modifications there.

![conf][conf1]
![conf][conf2]

Each cluster from step 2 is preprocessed into a histogram and scaled.
A feature obtained this way is used with the learned model to predict a label.
I publish a list of Detected Object that is later processed into YAML files using `instruction.py` node.

### Pick and Place Setup

#### 1. For all three tabletop setups (`test*.world`), perform object recognition, then read in respective pick list (`pick_list_*.yaml`). Next construct the messages that would comprise a valid `PickPlace` request output them to `.yaml` format.

The yaml files are in `data/` directory.
I've been able to successfully detect 100% of objects in each scene.

It's a bit disappointing that the normals features weren't useful at all.
I suspect this is because normals are sensitive to rotation.
Perhaps a larger training set or a feature that captures the relations between normals such as VFH estimation would be more effective.

![world1][world1]
![world2][world2]
![world3][world3]
