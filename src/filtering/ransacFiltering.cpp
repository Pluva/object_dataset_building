/** pcl **/
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

/** local **/
#include "../tools/customTypedef.hpp"
#include "../filtering/ransacFiltering.hpp"

/**
 * @RansacFiltering_computeMinVoxelsNB
 * Compute the minimal number of voxels a plan should contain to be removed.
 * @param
 * dw_x, dw_y, dw_z the parameters of the downsampling, therefore the volume "occupied" by a voxel is dw_x*dw_y*dw_z.
 * surfaceMin is the minimal surface in square of plan that should be removed.
 */
/**
 * @brief RansacFiltering_computeMinVoxelsNB Compute the minimal number of voxels a plan should contain to be removed.
 * @param dw_x
 * @param dw_y
 * @param dw_z
 * @param surfaceMin
 * @return
 */
int RansacFiltering_computeMinVoxelsNB(float dw_x, float dw_y, float dw_z, float surfaceMin)
{
    return (int) std::floor(surfaceMin / (dw_x * dw_x));
}


/**
 * @RansacFiltering
 * return value:
 *  0 : normal process, no plane removed, inputCloud = outputCloud
 *  1 : normal process, at least one plane removed, inputCloud.size > outputCloud.size
 */
/**
 * @brief RansacFiltering
 * @param inputCloud
 * @param outputCloud
 * @param planeMinFilteringSize
 * @param distanceThreshold
 * @return
 */
int RansacFiltering(PointCloudT::Ptr inputCloud, PointCloudT::Ptr outputCloud, int planeMinFilteringSize, float distanceThreshold)
{

    bool planeRemaining = true;                                                             // true while there is planes to remove
    PointCloudT::Ptr tempInputCloud = boost::make_shared <PointCloudT> (*inputCloud);       // local cloud used to run the algorithm
    PointCloudT::Ptr tempOutputCloud = boost::make_shared <PointCloudT> (*inputCloud);      // local cloud used to run the algorithm



    // SAC Segmentation
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<PointT> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (distanceThreshold);

    // While there is planes remaining (bigger than planeMinFilteringSize, keep removing them)
    while (planeRemaining)
    {
        seg.setInputCloud(tempInputCloud);
        seg.segment (*inliers, *coefficients);

        // Extract the planar inliers from the input cloud
        pcl::ExtractIndices<PointT> extract;
        extract.setInputCloud(tempInputCloud);
        extract.setIndices (inliers);
        extract.setNegative (true);

        extract.filter (*tempOutputCloud);

        // If the plane detected contains more voxels than the minimum required, then we remove it and cycle.
        if ((tempInputCloud->points.size() - tempOutputCloud->points.size() ) > planeMinFilteringSize)
        {
            pcl::copyPointCloud <PointT> (*tempOutputCloud, *tempInputCloud);
        }
        else
        {
            pcl::copyPointCloud <PointT> (*tempInputCloud, *outputCloud);
            planeRemaining = false;
        }
    }
    return (outputCloud->points.size() != inputCloud->points.size());
}
