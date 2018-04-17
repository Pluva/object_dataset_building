#ifndef pr2_vision_tools_hpp
#define pr2_vision_tools_hpp

/** std **/
#include <stdio.h>

/** opencv **/
#include <opencv/cv.hpp>

/** pcl **/
#include <pcl/range_image/range_image.h>
#include <pcl/segmentation/supervoxel_clustering.h>

/** image geometry **/
#include <image_geometry/pinhole_camera_model.h>

/** Local includes **/
#include "../tools/customTypedef.hpp"

/**
  *
  * /file pr2_vision_tools.hpp
  * @brief Library containing all functions related to image treatment.
  * /author Pluva
  *
  *
  */


/** VISION TOOLS **/
/**
    Library containing small functions related to image processing.
**/

// Usage needs to be cleaned up little by little to always make use of the namespace version of each function.
namespace VISION_TOOLS {

/**
 * @brief cloud_to_opencvMatImg
 * Convert a pcl PointCloudT into a cv::Mat image, image will be same dimension as input cloud (required to be organized).
 * @param cloud input source.
 * @return Resulting image.
 */
cv::Mat cloud_to_opencvMatImg(const PointCloudT::Ptr cloud);

/**
 * @brief projectPoint_3D_to_2D
 * Project a 3D point to the 2D plane given a camera transform.
 * @param pt_3D input 3D point to project.
 * @param camera_transform
 * @return Projected cv 2D point.
 */
cv::Point projectPoint_3D_to_2D(const PointT &pt_3D, const Eigen::MatrixXf &camera_transform);

/**
 * @brief projectPoint_3D_to_2D
 * @param pt_3d
 * @param pt_2d
 * @param cam_model
 */
void projectPoint_3D_to_2D(const PointT &pt_3d, cv::Point &pt_2d, const image_geometry::PinholeCameraModel &cam_model);

/**
 * @brief [Return a vector containing the rounded values of input vector. To be used in pair with projectPoint_3D_to_2D.]
 *
 * @param pt [Input 2D point]
 * @return [cv::Point = (round(pt.x), round(pt.y))]
 */
cv::Point round_2Dpoint(const cv::Point &pt);

/**
 * @brief [Combine both projection and rounding function, for commodity usage.]
 *
 * @param pt [3D point to project and round.]
 * @return [Rounded coordinates of Point pt projection.]
 */
cv::Point project_and_round(const cv::Point &pt, const Eigen::MatrixXf &camera_transform);
cv::Point project_and_round(const cv::Point &pt, const image_geometry::PinholeCameraModel &cam_model);

/**
 * @brief [Return true, if and only if, point pt is inside the frame.]
 *
 * @param pt [Input point]
 * @param frame_width [Frame width]
 * @param frame_height [Frame width]
 * @return [Return true if pt[0] in [0; frame_width[, pt[1] in [0; frame_height[]
 */
bool pointInsideFrame(const cv::Point &pt, int frame_width, int frame_height);

/**
 * @brief [Color the given coordinates pixel with the given rgb color.]
 *
 * @param image [Image under cv::Mat format.]
 * @param pt [Coordinates of the pixel to color. Coordinates at this point must be in format (abciss, ordinate) as they will be inversed INSIDE this function.]
 * @param r [Color, r value.]
 * @param g [Color, g value.]
 * @param b [Color, b value.]
 */
void colorPixelInFrame_safe(cv::Mat &image, const cv::Point &pt, int r, int g, int b);
void colorPixelInFrame_MONO8(cv::Mat &image, const cv::Point &pt, uchar intensity);

/**
 * @brief [Compute and return the max/min values of x and y axis amongst all voxels composing this object.]
 * @param obj [Source object hypothesis.]
 * @param coord [Output int[4] vector, structured as [minX, maxX, minY, maxY].]
 * @param camera_transform [Camera transformation to use to project the cloud points.]
 * @deprecated
 */
void computePatchCoordinates(const PointCloudT::Ptr rawCloud, Eigen::Vector4i &coord, const Eigen::MatrixXf &camera_transform, bool square = false);
/**
 * @brief computePatchCoordinates
 * @param rawCloud
 * @param coord
 * @param cam_model
 */
void computePatchCoordinates(const PointCloudT::Ptr rawCloud, Eigen::Vector4i &coord, const image_geometry::PinholeCameraModel &cam_model, bool square = false);

/**
 * @brief computePatchCoordinatesIntensity [Return the coordinates of the patches corresponding to objects as segmented by the intensity map.]
 * @param rawCloud [Intensity map]
 * @param coord [Coord object to fill]
 * @param cam_model [Camera parameters to project point cloud to 2D.]
 * @param square [Return a square patch]
 */
void computePatchCoordinates_Intensity(const PointCloudT::Ptr rawCloud, Eigen::Vector4i &coord, const image_geometry::PinholeCameraModel &cam_model, float intensity, float, bool square = false);

/**
 * @brief computeCloudSize
 * @param rawCloud
 * @param deltas
 */
void computeCloudSize(const PointCloudT::Ptr rawCloud, Eigen::Vector3f &deltas);

/**
 * @brief Extract a sub image from imageSource into imageTarget. Sub-image corresponds to square defined by (minX,maxX,minY,maxY).
 *
 * @param imageSource Source image for extraction.
 * @param imageTarget Target image for extraction.
 * @param patchCoord Eigen vector of dimension 4, structured as [minX, maxX, minY, maxY].
 * @param margeX Marge size along x axis in pixels.
 * @param margeY Marge size along y axis in pixels.
 * @param
 */
void extractPatchFromImage(const cv::Mat &imageSource, cv::Mat &imageTarget, const Eigen::Vector4i &patchCoord, int margeX, int margeY, int sourceWidth, int sourceHeight);

/**
 * @brief extractPatchFromImage
 * @param imageSource
 * @param imageTarget
 * @param rect
 * @param margeX
 * @param margeY
 * @param sourceWidth
 * @param sourceHeight
 */
void extractPatchFromImage(const cv::Mat &imageSource, cv::Mat &imageTarget, const cv::RotatedRect &rect, int margeX, int margeY, int sourceWidth, int sourceHeight);

/**
 * @brief randomizeLabels
 * @param cloud
 */
void randomizeLabels(PointLCloudT::Ptr cloud);

/**
 * @brief computeCloudAverageRGB
 * @param cloud
 */
void computeCloudAverageRGB(const PointCloudT::Ptr cloud, Eigen::Vector3i &rgb);
void computeCloudAverageRGB(const std::vector<pcl::Supervoxel<PointT>::Ptr> &sv, Eigen::Vector3i &rgb);

/**
 * @brief projectRawCloud
 * @param cloud
 * @param output_pts
 */
void projectCloud(const PointCloudT::Ptr cloud, const image_geometry::PinholeCameraModel &cam_model, std::vector<cv::Point> &output_pts);

/**
 * @brief projectSupervoxels
 * @param sv
 * @param output_pts
 */
void projectSupervoxels(const std::vector<pcl::Supervoxel<PointT>::Ptr> &sv, const image_geometry::PinholeCameraModel &cam_model, std::vector<cv::Point> &output_pts, double sv_radius);

/**
 * @brief computeBoundingRect.
 * Compute and return the minimum up-right bounding rectangle for the input set of points.
 * @param rawCloud
 * @param cam_model
 * @return
 */
cv::Rect computeBoundingRect(const PointCloudT::Ptr rawCloud, const image_geometry::PinholeCameraModel &cam_model);

/**
 * @brief computeBoundingRect.
 * Compute and return the minimum up-right bounding rectangle for the input set of supervoxels.
 * @param sv
 * @param cam_model
 * @return
 */
cv::Rect computeBoundingRect(const std::vector<pcl::Supervoxel<PointT>::Ptr> &sv, const image_geometry::PinholeCameraModel &cam_model, double sv_radius);
cv::Rect computeBoundingRect(const std::vector<cv::Point> projected_pts);

/**
 * @brief computeMinAreaRect.
 * Compute and return the minimum (possibly rotated) bounding rectangle for the given input set of points.
 * @param rawCloud
 * @param cam_model
 * @return
 */
cv::RotatedRect computeMinAreaRect(const PointCloudT::Ptr rawCloud, const image_geometry::PinholeCameraModel &cam_model);
/**
 * @brief computeMinAreaRect.
 * Compute and return the minimum (possibly rotated) bounding rectangle for the given input set of supervoxels.
 * @param sv
 * @param cam_model
 * @param sv_radius
 * @return
 */
cv::RotatedRect computeMinAreaRect(const std::vector<pcl::Supervoxel<PointT>::Ptr> &sv, const image_geometry::PinholeCameraModel &cam_model, double sv_radius);
cv::RotatedRect computeMinAreaRect(const std::vector<cv::Point> projected_pts);

/**
 * @brief computeConvexHull
 * @param rawCloud
 * @param cam_model
 * @param output_hull
 */
void computeConvexHull(const PointCloudT::Ptr rawCloud, const image_geometry::PinholeCameraModel &cam_model, std::vector<cv::Point> &output_hull);

/**
 * @brief computeConvexHull
 * @param sv
 * @param cam_model
 * @param output_hull
 */
void computeConvexHull(const std::vector<pcl::Supervoxel<PointT>::Ptr> &sv, const image_geometry::PinholeCameraModel &cam_model, double sv_radius, std::vector<cv::Point> &output_hull);
void computeConvexHull(const std::vector<cv::Point> projected_pts, std::vector<cv::Point> &output_hull);

/**
 * @brief selectPatchOnImage Return inside image_target a copy of image_source where every pixel not inside the given hull are set to black.
 * @param image_source
 * @param image_target
 * @param convex_hull
 */
void selectPatchOnImage(const cv::Mat &image_source, cv::Mat &image_target, const std::vector<cv::Point> &convex_hull);
void selectPatchOnImage(const cv::Mat &image_source, cv::Mat &image_target, const cv::RotatedRect &rect);

} // end of namespace VISION_TOOLS



#endif
