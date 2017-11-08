#ifndef pr2_ransacFiltering_hpp
#define pr2_ransacFiltering_hpp

/** pcl **/
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

/** local **/
#include "../tools/customTypedef.hpp"

int RansacFiltering_computeMinVoxelsNB(float dw_x, float dw_y, float dw_z, float surfaceMin);
int RansacFiltering(PointCloudT::Ptr inputCloud, PointCloudT::Ptr outputCloud, int planeMinFilteringSize, float distanceThreshold);

#endif
