#ifndef pr2_frustumFiltering_hpp
#define pr2_frustumFiltering_hpp

/** pcl **/
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/eigen.h>

/** local **/
#include "../tools/customTypedef.hpp"

void frustumFiltering_applyFilter(PointCloudT::Ptr inputCloud, PointCloudT::Ptr outputCloud, float verticlaFov, float horizontalFov, float nearPlaneDistance, float farPlaneDistance, Eigen::Matrix4f cameraPose);
void frustumFiltering_applyFilter_indexSafe(std::vector<int32_t> *oldIndices, PointCloudT::Ptr inputCloud, PointCloudT::Ptr outputCloud, float verticlaFov, float horizontalFov, float nearPlaneDistance, float farPlaneDistance, Eigen::Matrix4f cameraPose);


#endif
