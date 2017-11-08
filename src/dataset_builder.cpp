/** std **/
#include <vector>
#include <algorithm>
#include <string>
#include <sys/stat.h>
#include <sys/types.h>
#include <stdlib.h>

/** local **/
#include "tools/customTypedef.hpp"
#include "filtering/frustumFiltering.hpp"
#include "filtering/ransacFiltering.hpp"
#include "tools/customTypedef.hpp"
#include "tools/vision_tools.hpp"

/** opencv **/
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

/** pcl **/
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/pcl_base.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/transforms.h>

/** boost **/
#include <boost/filesystem.hpp>
namespace fs = boost::filesystem;
#include <boost/program_options.hpp>
namespace po = boost::program_options;

/** visualiwer TO REMOVE AFTER TEST **/
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>


/**
 * @brief ProcessData
 * @param input_cloud
 * @param input_image
 * @param output_cloud
 * @param output_image
 * @param fov_h
 * @param fov_w
 * @param fov_minDist
 * @param fov_maxDist
 * @param imageSize_h
 * @param imageSize_w
 * @param rf_tresh
 * @param rf_dist
 * @return
 */
int ProcessData(const PointCloudTA::Ptr input_cloud, const cv::Mat input_image, PointCloudTA::Ptr output_cloud, cv::Mat& output_image,
                const int fov_h, const int fov_w, const int fov_minDist, const int fov_maxDist,  const int imageSize_h, const int imageSize_w, const int rf_tresh, const float rf_dist)
{

    /** ALL THOSE PARAMETERS ARE CAMERA DEPENDENT **/
    /** You need to use those corresponding to the camera used to capture the images **/
    /** Maybe in the future those parameters should be placed and called in an extern file, to avoid recompiling **/
    // Pose of the camera in cartesian space
    Eigen::Matrix4f cameraPose;
    cameraPose <<   0, 0, 1, 0,
            0,-1, 0, 0,
            1, 0, 0, 0,
            0, 0, 0, 1;
    // Projection matrix
    Eigen::Matrix4f projectionMatrix;
    projectionMatrix << 1044.97, 0., 945, 0.,
            0., 1050.29, 553.36, 0.,
            0., 0., 1., 0.,
            0., 0., 0., 1.;
    /** ------------------------- **/

    /** **/
    /** REMOVING NaN AND INF POINTS **/
    /** **/
    std::vector<int> removedPointIndices;
    pcl::removeNaNFromPointCloud(*input_cloud, *output_cloud, removedPointIndices);
    /** **/
    /** ----------------------------------------------------- **/

    /** **/
    /** FILTERING CLOUD TRHOUGH FOV START **/
    /** **/
    frustumFiltering_applyFilter(output_cloud, output_cloud, fov_h, fov_w, fov_minDist, fov_maxDist, cameraPose);
    /** **/
    /** ----------------------------------------------------- **/

    /** **/
    /** FILTERING PLANE **/
    /** **/
    PointCloudTA::Ptr cloud_objects = boost::make_shared <PointCloudT> ();
    // USE - RansacFiltering_computeMinVoxelsNB(vgrid_x, vgrid_y, vgrid_z, 0.3) - IF DOWNSAMPLING, USE - 36000 - OTHERWISE.
    RansacFiltering(output_cloud, cloud_objects, rf_tresh, rf_dist);
    pcl::copyPointCloud(*cloud_objects, *output_cloud);
    /** ----------------------------------------------------- **/

    /** **/
    /** EXTRACTING PATCH FROM IMAGE **/
    /** **/
    Eigen::Vector4i coord;
    VISION_TOOLS::computePatchCoordinates(output_cloud, coord, projectionMatrix, true);
    VISION_TOOLS::extractPatchFromImage(input_image, output_image, coord, 15, 15, imageSize_w, imageSize_h);
    /** ----------------------------------------------------- **/

}

int main (int argc, char **argv)
{

    // Arguments
    std::string baseName_cloud;
    std::string baseName_image;
    std::string baseName_path;
    int fov_h;
    int fov_w;
    int fov_minDist;
    int fov_maxDist;
    int imageSize_h;
    int imageSize_w;
    int rf_tresh;
    float rf_dist;


    try
    {
        // Read arguments from config file
        po::options_description desc("Arguments");
        desc.add_options()
                ("baseName_cloud", po::value<std::string>(&baseName_cloud), "Cloud filename suffix")
                ("baseName_image", po::value<std::string>(&baseName_image), "Image filename suffix")
                ("baseName_path", po::value<std::string>(&baseName_path), "Path to the dataset")
                ("fov_h", po::value<int>(&fov_h), "Vertical fov for the frustum filtering")
                ("fov_w", po::value<int>(&fov_w), "Horizontal fov for the frustum filtering")
                ("fov_minDist", po::value<int>(&fov_minDist), "Minimal distance for the frustum filtering")
                ("fov_maxDist", po::value<int>(&fov_maxDist), "Maximal distance for the frustum filtering")
                ("imageSize_h", po::value<int>(&imageSize_h), "Image height")
                ("imageSize_w", po::value<int>(&imageSize_w), "Image width")
                ("rf_tresh", po::value<int>(&rf_tresh), "Ransac Filtering minimum number of voxels to be extract a plan")
                ("rf_dist", po::value<float>(&rf_dist), "Ransac Filtering distance to the plan to remove a point");
        po::variables_map vm;
        try
        {
            std::ifstream configfile(argv[1]);
            po::store(po::parse_command_line(argc, argv, desc), vm);
            po::store(po::parse_config_file(configfile, desc), vm);
            po::notify(vm);
        }
        catch(po::error& e)
        {
            std::cerr << "ERROR: " << e.what() << std::endl << std::endl;
            std::cerr << desc << std::endl;
            return 2;
        }

        // Go through the directory
        boost::filesystem::path p(baseName_path);
        fs::directory_iterator end_iter;
        for (fs::directory_iterator dir_itr(p); dir_itr != end_iter; ++dir_itr)
        {
            std::string currentFile = dir_itr->path().filename().string();
            if (currentFile.substr(currentFile.find("_") + 1) == "cloud.pcd")
            {
                // For each found cloud, process the data
                std::string currentCloud = baseName_path + currentFile;
                std::string currentImage = baseName_path + currentFile.substr(0, currentFile.find("_")) + "_color.jpg";
                std::cout << "-- Processing:" << std::endl;
                std::cout << "---- cloud : " + currentCloud << std::endl;
                std::cout << "---- image : " + currentImage << std::endl;

                // Load the cloud
                PointCloudTA::Ptr loaded_cloud (new PointCloudTA);
                if (pcl::io::loadPCDFile<PointTA>(baseName_path + currentFile, *loaded_cloud) == -1)
                {
                    PCL_ERROR ("Couldn't read cloud file\n");
                    return (-1);
                }

                // Load the 2D image
                cv::Mat loaded_image = cv::imread(currentImage, CV_LOAD_IMAGE_COLOR);
                if (loaded_image.data == NULL)
                {
                    std::cerr << "Couldn't read image file\n" << std::endl;
                    return (-1);
                }

                // Process Data to create a usable image and cloud of this object.
                // -- This process, currently assumes that there is only one 'object' on the table.
                // -- So that by reducing FoV, and removing planes, only one 'object' should remain.
                // -- Therefore, as of now, this method isn't suitable for any other situation.
                PointCloudTA::Ptr processed_cloud (new PointCloudTA);
                cv::Mat processed_image;
                ProcessData(loaded_cloud, loaded_image, processed_cloud, processed_image, fov_h, fov_w, fov_minDist, fov_maxDist, imageSize_h, imageSize_w, rf_tresh, rf_dist);

                // Save Data
                mkdir((baseName_path + "build/").c_str(), 0777);
                std::cout << "Saving the cloud in: " + baseName_path + "build/_processed_" + currentFile << std::endl;
                pcl::io::savePCDFileASCII (baseName_path + "build/_processed_" + currentFile, *processed_cloud);
                std::cout << "Saving the image in: " + baseName_path + "build/_processed_" + "_" + currentFile.substr(0, currentFile.find("_")) + "_color.jpg" << std::endl;
                cv::imwrite(baseName_path + "build/_processed_" + "_" + currentFile.substr(0, currentFile.find("_")) + "_color.jpg", processed_image);
            }
        }

        //    // Cloud Visualisation
        //    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
        //    viewer->setBackgroundColor (0, 0, 0);
        //    pcl::visualization::PointCloudColorHandlerRGBField<PointTA> rgb(processed_cloud);
        //    viewer->addPointCloud<PointTA> (processed_cloud, rgb, "sample cloud");
        //    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
        //    viewer->addCoordinateSystem (1.0);
        //    viewer->initCameraParameters ();
        //    while (!viewer->wasStopped ())
        //    {
        //        viewer->spinOnce (100);
        //        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
        //    }

    }
    catch(std::exception& e)
    {
        std::cerr << "Unhandled Exception" << e.what() << ", application will now exit" << std::endl;
        return 2;
    }

    return 0;
}
