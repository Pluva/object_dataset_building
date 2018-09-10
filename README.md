# object_dataset_building
Basic code to manipulate raw RGB-D camera data (images and cloud) and create usable datasets.
One function to segment foreground objects from the background, and produce corresponding subsets.
One function to evaluate a cloud using a relevance map (Leni Le Goff's babbling https://github.com/robotsthatdream/saliency_map_babbling).

## dataset_builder:
### Modes:
#### Relevance Map:
Process a dataset of images with a given RelevanceMap. Produces a 2D patch and a corresponding label for each supervoxels, as evaluated by the relevance map.

Usage:
> ./dataset_builder rm path_to_config_file

Config File Format:
```
[Arguments]
baseName_cloud = _cloud.pcd           # suffix of cloud files
baseName_image = _color.jpg           # suffix of image files
baseName_path = /path/to/data/        # path to the dataset
imageSize_h = 1080                    # expected size of images to process
imageSize_w = 1920  
cic_x_max = 0.2                       # workspace parameters, see code for details
cic_x_min = -0.4
cic_y_max = 0.5
cic_y_min = -0.05
cic_z_max = 3
cic_z_min = 0.85
cic_x_marge = 3
cic_y_marge = 3
classifier_archive = /path/to/archive # trained relevance map archive
```

#### Object Segmentation:
Segment salliant objects from the background using simple planar extraction.
Produces a set of sub clouds and images for each objects segmented that way.

Usage:
> ./dataset_builder os path_to_config_file

Config File Format:
```
baseName_cloud=_cloud.pcd                     # suffix of cloud files
baseName_image=_color.jpg                     # suffix of image files
baseName_path=/path/to/dataset                # path to dataset
fov_h=20                                      # field of view to consider
fov_w=20
fov_minDist=0
fov_maxDist=1
imageSize_h=1080                              # expected image size
imageSize_w=1920
rf_tresh=35000                                # ransac parameters
rf_dist=0.012
```

## dataset_tools:
