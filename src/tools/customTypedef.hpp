#ifndef CUSTOM_TYPEDEF_HPP
#define CUSTOM_TYPEDEF_HPP

/** std **/
#include <set>
#include <vector>

/** pcl **/
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

/** PCL TYPEDEF **/
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointXYZRGBA PointTA;
typedef pcl::PointCloud<PointTA> PointCloudTA;
typedef pcl::PointXYZRGB PointTnA;
typedef pcl::PointCloud<PointTnA> PointCloudTnA;
typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointNT> PointNCloudT;
typedef pcl::PointXYZL PointLT;
typedef pcl::PointCloud<PointLT> PointLCloudT;


/** VECTOR TYPEDEF **/
typedef std::vector<int32_t> vect_int;
typedef std::vector<PointT> vect_ptT;
typedef std::vector<std::set<int32_t> > vect_set;


#endif
