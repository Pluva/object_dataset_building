/** GENERAL INCLUDES **/
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/frustum_culling.h>
#include <pcl/common/eigen.h>


/** LOCAL INCLUDES **/
#include "../filtering/frustumFiltering.hpp"
#include "../tools/customTypedef.hpp"

/**
 * @brief [Apply frustum culling to the input cloud.]
 * 
 * @param inputCloud [description]
 * @param outputCloud [description]
 * @param verticalFov [description]
 * @param horizontalFov [description]
 * @param nearPlaneDistance [description]
 * @param farPlaneDistance [description]
 * @param cameraPose [description]
 */
void frustumFiltering_applyFilter(PointCloudT::Ptr inputCloud, PointCloudT::Ptr outputCloud, float verticalFov, float horizontalFov, float nearPlaneDistance, float farPlaneDistance, Eigen::Matrix4f cameraPose)
{
    // CREATE FILTERING OBJECT
    pcl::FrustumCulling<PointT> fc;
    // SET PARAMETERS
    fc.setInputCloud (inputCloud);
    fc.setVerticalFOV (verticalFov);
    fc.setHorizontalFOV (horizontalFov);
    fc.setNearPlaneDistance (nearPlaneDistance);
    fc.setFarPlaneDistance (farPlaneDistance);
    fc.setCameraPose(cameraPose);
    // FILTER
    fc.filter(*outputCloud);
}

/**
 * @brief [Apply frustum culling to the input cloud, keep track of indices inside oldIndices vector.]
 * 
 * @param oldIndices [description]
 * @param inputCloud [description]
 * @param outputCloud [description]
 * @param verticalFov [description]
 * @param horizontalFov [description]
 * @param nearPlaneDistance [description]
 * @param farPlaneDistance [description]
 * @param cameraPose [description]
 */
void frustumFiltering_applyFilter_indexSafe(std::vector<int32_t> *oldIndices, PointCloudT::Ptr inputCloud, PointCloudT::Ptr outputCloud, float verticalFov, float horizontalFov, float nearPlaneDistance, float farPlaneDistance, Eigen::Matrix4f cameraPose)
{
    // CREATE FILTERING OBJECT
    pcl::FrustumCulling<PointT> fc;
    // SET PARAMETERS
    fc.setInputCloud (inputCloud);
    fc.setVerticalFOV (verticalFov);
    fc.setHorizontalFOV (horizontalFov);
    fc.setNearPlaneDistance (nearPlaneDistance);
    fc.setFarPlaneDistance (farPlaneDistance);
    fc.setCameraPose(cameraPose);
    // FILTER

    fc.filter(*oldIndices);

    for (int i = 0; i < oldIndices->size(); i++)
    {
        outputCloud->push_back(inputCloud->points[(*oldIndices)[i]]);
    }

}
