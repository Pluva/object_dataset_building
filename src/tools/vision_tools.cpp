/** std **/
#include <cstdlib>

/** opencv **/
// opencv bug work around
// no other better way ?
// Required ?! ->
// #include <vector>
// using namespace std;
#include <vector>
using namespace std;
// ----------------------

/** pcl **/
#include <pcl/range_image/range_image.h>

/** Local includes **/
#include "../tools/vision_tools.hpp"







/** NAMESPACE VERSION **/
// In term all call to vision tools functions should go through the namespace.



namespace VISION_TOOLS {




/**
 * @brief cloud_to_opencvMatImg
 * @param cloud
 * @return
 */
cv::Mat cloud_to_opencvMatImg(const PointCloudT::Ptr cloud)
{

    if (!cloud->isOrganized())
    {
        throw std::invalid_argument("Input cloud must be organized !");
    }

    cv::Mat image(cloud->height, cloud->width, CV_8UC3, cv::Scalar(255,0,0));
    PointT *pt;

    std::cout << "Step1\n";
    // cv::Scalar<CV_8UC3> *color;

    for (int w = 0; w < cloud->height; w++)
    {
        for (int h = 0; h < cloud->height; h++)
        {
            // std::cout << "---------------- " << cloud->width << " " << cloud->height << " " << w << " " << h << " \n";
            pt = &((*cloud)[w + h * cloud->width]);
            // std::cout << "---------------- " << pt->r << " " << pt->g << " " << pt->b << " \n";
            image.at<cv::Scalar>(h, w) = cv::Scalar(((pt->r <= 255)&&(pt->r >= 0)) ? pt->r : 0 , ((pt->g <= 255)&&(pt->g >= 0)) ? pt->g : 0, ((pt->b <= 255)&&(pt->b >= 0)) ? pt->b : 0);
        }
    }

    // cv::initModule_nonfree();
    std::cout << "Step2\n";

    return image;

}

/**
 * @brief [Project coordinates 3D point pt_3D, given a camera pose.]
 * @details [long description]
 *
 * @param pt_3D [3D point to project]
 * @param camera_transform [Camera transformation used for projection. Must be of dimension 3x4.]
 * @return[Vector of 2D point coordinates.]
 */
cv::Point projectPoint_3D_to_2D(const PointT &pt_3D, const Eigen::MatrixXf &camera_transform)
{
    Eigen::Vector4f pt(pt_3D.x, pt_3D.y, pt_3D.z, 1);
    Eigen::Vector4f vec = camera_transform * pt;
    //std::printf("PROJECTING POINT (%f, %f, %f) to (%f, %f, %f) and finally (%f, %f)", pt_3D.x, pt_3D.y, pt_3D.z, vec[0], vec[1], vec[2], vec[0] / vec[2], vec[1] / vec[2]);
    return cv::Point(vec[0] / vec[2], vec[1] / vec[2]);
}

void projectPoint_3D_to_2D(const PointT &pt_3d, cv::Point &pt_2d, const image_geometry::PinholeCameraModel &cam_model)
{
    cv::Point3d cv_pt_3d;
    cv_pt_3d.x = pt_3d.x;
    cv_pt_3d.y = pt_3d.y;
    cv_pt_3d.z = pt_3d.z;
    pt_2d = cam_model.project3dToPixel(cv_pt_3d);
}

/**
 * @brief [Return a vector containing the rounded values of input vector. To be used in pair with projectPoint_3D_to_2D.]
 *
 * @param pt [Input 2D point]
 * @return [cv::Point = (round(pt.x), round(pt.y))]
 */
cv::Point round_2Dpoint(const cv::Point &pt)
{
    return cv::Point(round(pt.x), round(pt.y));
}

/**
 * @brief [Combine both projection and rounding function, for commodity usage.]
 *
 * @param pt [3D point to project and round.]
 * @return [Rounded coordinates of Point pt projection.]
 */
cv::Point project_and_round(const PointT &pt_3D, const Eigen::MatrixXf &camera_transform)
{
    return round_2Dpoint(projectPoint_3D_to_2D(pt_3D, camera_transform));
}
cv::Point project_and_round(const PointT &pt_3D, const image_geometry::PinholeCameraModel &cam_model)
{
    cv::Point pt_2d;
    projectPoint_3D_to_2D(pt_3D, pt_2d, cam_model);
    return pt_2d;
    //    return round_2Dpoint(pt_2d);
}

/**
 * @brief [Return true, if and only if, point pt is inside the frame.]
 *
 * @param pt [Input point]
 * @param frame_width [Frame width]
 * @param frame_height [Frame width]
 * @return [Return true if pt[0] in [0; frame_width[, pt[1] in [0; frame_height[]
 */
bool pointInsideFrame(const cv::Point &pt, int frame_width, int frame_height)
{
    return ((pt.x < frame_width) && (pt.x >= 0) && (pt.y < frame_height) && (pt.y >= 0));
}

/**
 * @brief colorPixelInFrame_safe Color given pixel within given frame with given rgb color (boundary safe version).
 * @param image
 * @param pt
 * @param r
 * @param g
 * @param b
 */
void colorPixelInFrame_safe(cv::Mat &image, const cv::Point &pt, int r, int g, int b)
{
    if ((pt.y >= image.rows) || (pt.y < 0) || (pt.x >= image.cols) || (pt.x < 0))
    {
        throw std::invalid_argument("The pixel to paint must be inside the frame !");
    }
    image.at<cv::Scalar>(cv::Point(pt.y, pt.x)) = cv::Scalar(r, g, b);
}

/**
 * @brief colorPixelInFrame_MONO8 Color given pixel within given frame with given uchar intesity color value (boundary unsafe version).
 * @param image
 * @param pt
 * @param intensity
 */
void colorPixelInFrame_MONO8(cv::Mat &image, const cv::Point &pt, uchar intensity)
{
    image.at<uchar>(pt) = intensity;
    // image.at<uchar>(cv::Point(pt.y, pt.x)) = intensity;
}


//void computePatchCoordinates(PR2_objectHypothesis &obj, Eigen::Vector4i &coord, const Eigen::MatrixXf &camera_transform)
//{
//    PointCloudT::Ptr tmpCloud = boost::make_shared<PointCloudT>();
//    cv::Point tmpPt;

//    // Get the voxels cloud
//    obj.getRawData()->getRawCloudCopy(tmpCloud);

//    // Initialization
//    tmpPt = project_and_round(tmpCloud->points[0], camera_transform);
//    coord[0] = tmpPt.x;
//    coord[1] = tmpPt.x;
//    coord[2] = tmpPt.y;
//    coord[3] = tmpPt.y;

//    // Iterate on every voxels of the cloud.
//    for (PointCloudT::iterator pts = tmpCloud->begin(); pts != tmpCloud->end(); ++pts)
//    {
//        tmpPt = project_and_round(*pts, camera_transform);
//        coord[0] = std::min(coord[0], tmpPt.x);
//        coord[1] = std::max(coord[1], tmpPt.x);
//        coord[2] = std::min(coord[2], tmpPt.y);
//        coord[3] = std::max(coord[3], tmpPt.y);
//    }
//}

void computePatchCoordinates(const PointCloudT::Ptr rawCloud, Eigen::Vector4i &coord, const Eigen::MatrixXf &camera_transform, bool square)
{
    cv::Point tmpPt;
    // Initialization
    tmpPt = project_and_round(rawCloud->points[0], camera_transform);
    coord[0] = tmpPt.x;
    coord[1] = tmpPt.x;
    coord[2] = tmpPt.y;
    coord[3] = tmpPt.y;
    // Iterate on every voxels of the cloud.
    for (PointCloudT::iterator pts = rawCloud->begin(); pts != rawCloud->end(); ++pts)
    {
        tmpPt = project_and_round(*pts, camera_transform);
        coord[0] = std::min(coord[0], tmpPt.x);
        coord[1] = std::max(coord[1], tmpPt.x);
        coord[2] = std::min(coord[2], tmpPt.y);
        coord[3] = std::max(coord[3], tmpPt.y);
    }
    // Make the patch square (for further processing requiring images of same dimensions.
    if (square)
    {
        int width = coord[1] - coord[0];
        int height = coord[3] - coord[2];
        std::cout << height << " " << width << std::endl;
        if (height > width)
        {
            coord[0] -= (height - width)/2;
            coord[1] += (height - width)/2 + (height - width)%2;
        }
        else if (height < width)
        {
            coord[2] -= (width - height)/2;
            coord[3] += (width - height)/2 + (width - height)%2;
        }
        else
        {
            // Do nothing
        }
    }
}
void computePatchCoordinates(const PointCloudT::Ptr rawCloud, Eigen::Vector4i &coord, const image_geometry::PinholeCameraModel &cam_model, bool square)
{
    cv::Point tmpPt;
    // Initialization
    tmpPt = project_and_round(rawCloud->points[0], cam_model);
    coord[0] = tmpPt.x;
    coord[1] = tmpPt.x;
    coord[2] = tmpPt.y;
    coord[3] = tmpPt.y;
    // Iterate on every voxels of the cloud.
    for (PointCloudT::iterator pts = rawCloud->begin(); pts != rawCloud->end(); ++pts)
    {
        tmpPt = project_and_round(*pts, cam_model);
        coord[0] = std::min(coord[0], tmpPt.x);
        coord[1] = std::max(coord[1], tmpPt.x);
        coord[2] = std::min(coord[2], tmpPt.y);
        coord[3] = std::max(coord[3], tmpPt.y);
    }
    // Make the patch square (for further processing requiring images of same dimensions.
    if (square)
    {
        int width = coord[1] - coord[0];
        int height = coord[3] - coord[2];
        std::cout << height << " " << width << std::endl;
        if (height > width)
        {
            coord[0] -= (height - width)/2;
            coord[1] += (height - width)/2 + (height - width)%2;
        }
        else if (height < width)
        {
            coord[2] -= (width - height)/2;
            coord[3] += (width - height)/2 + (width - height)%2;
        }
        else
        {
            // Do nothing
        }
    }
}

//void computeCloudSize(PR2_objectHypothesis &obj, Eigen::Vector3f &deltas)
//{
//    float min_x;
//    float max_x;
//    float min_y;
//    float max_y;
//    float min_z;
//    float max_z;

//    PointCloudT::Ptr tmpCloud = boost::make_shared<PointCloudT>();

//    // Get the voxels cloud
//    obj.getRawData()->getRawCloudCopy(tmpCloud);

//    // Initialization
//    min_x = max_x = tmpCloud->points[0].x;
//    min_y = max_y = tmpCloud->points[0].y;
//    min_z = max_z = tmpCloud->points[0].z;

//    // Iterate on every voxels of the cloud.
//    for (PointCloudT::iterator pts = tmpCloud->begin(); pts != tmpCloud->end(); ++pts)
//    {
//        min_x = std::min(min_x, pts->x);
//        max_x = std::max(max_x, pts->x);
//        min_y = std::min(min_y, pts->y);
//        max_y = std::max(max_y, pts->y);
//        min_z = std::min(min_z, pts->z);
//        max_z = std::max(max_z, pts->z);
//    }
//    deltas[0] = max_x - min_x;
//    deltas[1] = max_y - min_y;
//    deltas[2] = max_z - min_z;
//}
void computeCloudSize(PointCloudT::Ptr rawCloud, Eigen::Vector3f &deltas)
{
    float min_x;
    float max_x;
    float min_y;
    float max_y;
    float min_z;
    float max_z;

    // Initialization
    min_x = max_x = rawCloud->points[0].x;
    min_y = max_y = rawCloud->points[0].y;
    min_z = max_z = rawCloud->points[0].z;

    // Iterate on every voxels of the cloud.
    for (PointCloudT::iterator pts = rawCloud->begin(); pts != rawCloud->end(); ++pts)
    {
        min_x = std::min(min_x, pts->x);
        max_x = std::max(max_x, pts->x);
        min_y = std::min(min_y, pts->y);
        max_y = std::max(max_y, pts->y);
        min_z = std::min(min_z, pts->z);
        max_z = std::max(max_z, pts->z);
    }
    deltas[0] = max_x - min_x;
    deltas[1] = max_y - min_y;
    deltas[2] = max_z - min_z;
}

void extractPatchFromImage(const cv::Mat &imageSource, cv::Mat &imageTarget, const Eigen::Vector4i &patchCoord, int margeX, int margeY, int sourceWidth, int sourceHeight)
{

    int x, y, w, h;
    x = std::max(patchCoord[0] - margeX, 0);
    y = std::max(patchCoord[2] - margeY, 0);
    w = (patchCoord[1] + margeX) >= sourceWidth ? (sourceWidth - x) : (patchCoord[1] - x + margeX);
    h = (patchCoord[3] + margeY) >= sourceHeight ? (sourceHeight - y) : (patchCoord[3] - y + margeY);
    cv::Rect patchRect(x, y, w, h);
    cv::Mat patchImage = imageSource(patchRect);
    patchImage.copyTo(imageTarget);
}



void randomizeLabels(PointLCloudT::Ptr cloud)
{
    srand(time(NULL));
    std::map<uint32_t, uint32_t> new_labels;
    for (PointLCloudT::iterator pts = cloud->begin(); pts != cloud->end(); ++pts)
    {
        if (new_labels.find(pts->label) == new_labels.end())
        {
            new_labels.insert(std::pair<uint32_t, uint32_t>(pts->label, ((uint32_t) rand()) % 10000 ));
            pts->label = new_labels[pts->label];
        } else
        {
            pts->label = new_labels[pts->label];
        }
    }
}

void computeCloudAverageRGB(const PointCloudT::Ptr cloud, Eigen::Vector3i &rgb)
{
    int r = 0, g = 0, b = 0;
    uint32_t nb_pts = 0; // Compatibility ? Width and size seems to not be consistent through versions.
    for (PointCloudT::iterator pts = cloud->begin(); pts != cloud->end(); ++pts)
    {
        r += pts->r;
        g += pts->g;
        b += pts->b;
        nb_pts++;
    }
    rgb[0] = r / nb_pts;
    rgb[1] = g / nb_pts;
    rgb[2] = b / nb_pts;
}
void computeCloudAverageRGB(const std::vector<pcl::Supervoxel<PointT>::Ptr> &sv, Eigen::Vector3i &rgb)
{
    int r = 0, g = 0, b = 0;
    for (std::vector<pcl::Supervoxel<PointT>::Ptr>::const_iterator svs = sv.begin(); svs != sv.end(); ++svs)
    {
        r += (*svs)->centroid_.r;
        g += (*svs)->centroid_.g;
        b += (*svs)->centroid_.b;
    }
    rgb[0] = r / sv.size();
    rgb[1] = g / sv.size();
    rgb[2] = b / sv.size();
}


void projectCloud(const PointCloudT::Ptr cloud, const image_geometry::PinholeCameraModel &cam_model, std::vector<cv::Point> &output_pts)
{
    output_pts.clear();
    cv::Point pt;
    for (PointCloudT::iterator pts = cloud->begin(); pts != cloud->end(); ++pts)
    {
        projectPoint_3D_to_2D(*pts, pt, cam_model);
        output_pts.push_back(cv::Point(pt));
    }
}

void projectSupervoxels(const std::vector<pcl::Supervoxel<PointT>::Ptr> &sv, const image_geometry::PinholeCameraModel &cam_model, std::vector<cv::Point> &output_pts, double sv_radius)
{
    output_pts.clear();
    cv::Point pt;
    for (std::vector<pcl::Supervoxel<PointT>::Ptr>::const_iterator svs = sv.begin(); svs != sv.end(); ++svs)
    {
        projectPoint_3D_to_2D((*svs)->centroid_, pt, cam_model);
        output_pts.push_back(cv::Point(pt));
        pt.x += sv_radius; output_pts.push_back(cv::Point(pt));
        pt.x -= sv_radius; output_pts.push_back(cv::Point(pt));
        pt.y += sv_radius; output_pts.push_back(cv::Point(pt));
        pt.y -= sv_radius; output_pts.push_back(cv::Point(pt));
    }
}

cv::Rect computeBoundingRect(const PointCloudT::Ptr rawCloud, const image_geometry::PinholeCameraModel &cam_model)
{
    std::vector<cv::Point> projected_cloud;
    projectCloud(rawCloud, cam_model, projected_cloud);
    return cv::boundingRect(projected_cloud);
}

cv::Rect computeBoundingRect(const std::vector<pcl::Supervoxel<PointT>::Ptr> &sv, const image_geometry::PinholeCameraModel &cam_model, double sv_radius)
{
    std::vector<cv::Point> projected_cloud;
    projectSupervoxels(sv, cam_model, projected_cloud, sv_radius);
    return cv::boundingRect(projected_cloud);
}

cv::Rect computeBoundingRect(const std::vector<cv::Point> projected_pts)
{
    return cv::boundingRect(projected_pts);
}

cv::RotatedRect computeMinAreaRect(const PointCloudT::Ptr rawCloud, const image_geometry::PinholeCameraModel &cam_model)
{
    std::vector<cv::Point> projected_cloud;
    projectCloud(rawCloud, cam_model, projected_cloud);
    return cv::minAreaRect(projected_cloud);
}

cv::RotatedRect computeMinAreaRect(const std::vector<pcl::Supervoxel<PointT>::Ptr> &sv, const image_geometry::PinholeCameraModel &cam_model, double sv_radius)
{
    std::vector<cv::Point> projected_cloud;
    projectSupervoxels(sv, cam_model, projected_cloud, sv_radius);
    return cv::minAreaRect(projected_cloud);
}

cv::RotatedRect computeMinAreaRect(const std::vector<cv::Point> projected_pts)
{
    return cv::minAreaRect(projected_pts);
}

void computeConvexHull(const PointCloudT::Ptr rawCloud, const image_geometry::PinholeCameraModel &cam_model, std::vector<cv::Point> &output_hull)
{
    std::vector<cv::Point> projected_cloud;
    projectCloud(rawCloud, cam_model, projected_cloud);
    cv::convexHull(projected_cloud, output_hull);
}

void computeConvexHull(const std::vector<pcl::Supervoxel<PointT>::Ptr> &sv, const image_geometry::PinholeCameraModel &cam_model, double sv_radius, std::vector<cv::Point> &output_hull)
{
    std::vector<cv::Point> projected_cloud;
    projectSupervoxels(sv, cam_model, projected_cloud, sv_radius);
    cv::convexHull(projected_cloud, output_hull);
}

void computeConvexHull(const std::vector<cv::Point> projected_pts, std::vector<cv::Point> &output_hull)
{
    cv::convexHull(projected_pts, output_hull);
}

void selectPatchOnImage(const cv::Mat &image_source, cv::Mat &image_target, const std::vector<cv::Point> &convex_hull)
{
    cv::Mat mask(image_source.rows, image_source.cols, CV_8U, cv::Scalar(0));
    cv::Point hull[1][convex_hull.size()];
    int i = 0;
    for (std::vector<cv::Point>::const_iterator pt = convex_hull.begin(); pt != convex_hull.end(); ++pt)
    {
        hull[0][i++] = cv::Point(*pt);
    }
    const cv::Point * ppt[1] = { hull[0] };
    int npt[] = { convex_hull.size() };
    cv::fillPoly(mask, ppt, npt, 1, cv::Scalar(255));
    image_source.copyTo(image_target, mask);
}

}
