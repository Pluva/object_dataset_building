/** std **/
#include <vector>
#include <algorithm>
#include <iostream>
#include <fstream>
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

/** relevance map related libraries **/
#include <iagmm/gmm.hpp>
#include <image_processing/SurfaceOfInterest.h>
namespace ip = image_processing;
#include <boost/archive/text_iarchive.hpp>


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
int ObjectProcessing(const PointCloudTA::Ptr input_cloud, const cv::Mat input_image, PointCloudTA::Ptr output_cloud, cv::Mat& output_image,
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
    // OLD ?
    Eigen::Matrix4f projectionMatrix;
    projectionMatrix << 1044.97, 0., 945, 0.,
            0., 1050.29, 553.36, 0.,
            0., 0., 1., 0.,
            0., 0., 0., 1.;
//    Eigen::Matrix4f projectionMatrix;
//    projectionMatrix << 1051.77, 0.0, 927.93, 0.0,
//            0.0, 1051.76, 516.40, 0.0,
//            0.0, 0.0, 1.0, 0.0;
//            0., 0., 0., 1.;

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

/**
 * @brief RelevanceMapProcessing
 * @param input_cloud
 * @param input_image
 * @param output_cloud
 * @param output_image
 * @param classifier_archive
 * @param x_min
 * @param x_max
 * @param y_min
 * @param y_max
 * @param z_min
 * @param z_max
 * @param imageSize_h
 * @param imageSize_w
 * @param certainty_treshold
 */
void RelevanceMapProcessing(const PointCloudTA::Ptr input_cloud, const cv::Mat input_image,
                            std::vector<std::pair<cv::Mat, double>>& output_images, const std::string classifier_archive,
                            const float x_min , const float x_max,
                            const float y_min, const float y_max,
                            const float z_min, const float z_max,
                            const int marge_x, const int marge_y,
                            const int imageSize_h, const int imageSize_w)
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
    // PR2 Calib
    Eigen::Matrix4f projectionMatrix;
    projectionMatrix << 1051.77, 0.0, 927.93, 0.0,
            0.0, 1051.76, 516.40, 0.0,
            0.0, 0.0, 1.0, 0.0;
            0., 0., 0., 1.;

    /** ------------------------- **/

    /** Generating Relevance Map **/

    //* load classifier from an archive
    iagmm::GMM classifier;
    std::ifstream ifs(classifier_archive);
    if(!ifs){
        std::cerr << "unable to open gmm classifier archive" << std::endl;
        return;
    }
    boost::archive::text_iarchive iarch(ifs);
    iarch >> classifier;
    //*/

    //* computing relevance map
    ip::SurfaceOfInterest soi;
    soi.setInputCloud(input_cloud);
    std::unique_ptr<ip::workspace_t> workspace(
                new ip::workspace_t(false,0,0,0,0,0,
                {x_min,x_max,y_min,y_max,z_min,z_max}));

    soi.computeSupervoxel(*workspace);
    std::cout << "nbr supervoxels "  << soi.getSupervoxels().size() << std::endl;
    soi.compute_feature("meanFPFHLabHist");
    soi.compute_weights<iagmm::GMM>("meanFPFHLabHist",classifier);

    // Get the labels of each supervoxels
    std::map<uint32_t,double> relevance_map = soi.get_weights()["meanFPFHLabHist"];

    // Get the intensity map
//    pcl::PointCloud<pcl::PointXYZI> relevance_map_cloud = soi.getColoredWeightedCloud("meanFPFHLabHist");


    // Get access to the supervoxels

    /** Extracting patches **/
    // Supervoxels patches

    Eigen::Vector4i bounding_rect;
    cv::Mat output_image;
    for(const auto& sv : soi.getSupervoxels()){

//        it->first; //label of the supervoxel (uint32)
//        it->second; //supervoxel itself

        double weight = relevance_map[sv.first]; // get map value for this SV

        if (!sv.second->voxels_->empty())
        {
            // Extract patch from image
            VISION_TOOLS::computePatchCoordinates(sv.second->voxels_, bounding_rect, projectionMatrix, true);
            VISION_TOOLS::extractPatchFromImage(input_image, output_image, bounding_rect, marge_x, marge_y, imageSize_w, imageSize_h);
            output_images.push_back(std::make_pair(cv::Mat(output_image), weight));
        }
    }

    // Object patches

    /** ----------------------------------------------------- **/

}

/** Loop over dataset **/

/**
 * @brief Main function, browse through the repertory given as parameter in the config file and create according dataset.
 * Creates patches of images according to the corresponding clouds.
 * @param argc
 * @param argv
 * @return
 */
int LoopOverDataset_ProcessData(int argc, char **argv)
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
            std::ifstream configfile(argv[0]);
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
                ObjectProcessing(loaded_cloud, loaded_image, processed_cloud, processed_image, fov_h, fov_w, fov_minDist, fov_maxDist, imageSize_h, imageSize_w, rf_tresh, rf_dist);


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

/**
 * @brief Main function, browse through the repertory given as parameter in the config file and create according dataset.
 * @param argc
 * @param argv
 * @return
 */
int LoopOverDataset_RelevanceMap(int argc, char **argv)
{

    // Arguments
    std::string baseName_cloud;
    std::string baseName_image;
    std::string baseName_path;

    int imageSize_h;
    int imageSize_w;

    float cic_x_min;
    float cic_x_max;
    float cic_y_min;
    float cic_y_max;
    float cic_z_min;
    float cic_z_max;

    float cic_x_marge;
    float cic_y_marge;

    std::string classifier_archive;

    try
    {
        // Read arguments from config file
        po::options_description desc("Arguments");
        desc.add_options()
                ("Arguments.baseName_cloud", po::value<std::string>(&baseName_cloud), "Cloud filename suffix")
                ("Arguments.baseName_image", po::value<std::string>(&baseName_image), "Image filename suffix")
                ("Arguments.baseName_path", po::value<std::string>(&baseName_path), "Path to the dataset")
                ("Arguments.imageSize_h", po::value<int>(&imageSize_h), "Image height")
                ("Arguments.imageSize_w", po::value<int>(&imageSize_w), "Image width")
                ("Arguments.cic_x_min", po::value<float>(&cic_x_min))
                ("Arguments.cic_x_max", po::value<float>(&cic_x_max))
                ("Arguments.cic_y_min", po::value<float>(&cic_y_min))
                ("Arguments.cic_y_max", po::value<float>(&cic_y_max))
                ("Arguments.cic_z_min", po::value<float>(&cic_z_min))
                ("Arguments.cic_z_max", po::value<float>(&cic_z_max))
                ("Arguments.cic_x_marge", po::value<float>(&cic_x_marge), "Marge over the x axis (width)")
                ("Arguments.cic_y_marge", po::value<float>(&cic_y_marge), "Marge over the y axis (height)")
                ("Arguments.classifier_archive", po::value<std::string>(&classifier_archive));
        po::variables_map vm;






        try
        {
            // Parse command options
            std::ifstream configfile(argv[0]);
            po::store(po::parse_command_line(argc, argv, desc), vm);
            po::store(po::parse_config_file(configfile, desc), vm);
            po::notify(vm);

//            configfile.close();
        }
        catch(po::error& e)
        {
            std::cerr << "ERROR: " << e.what() << std::endl << std::endl;
            std::cerr << desc << std::endl;
            return 2;
        }

        // Create directory for created data.
        mkdir((baseName_path + "build/").c_str(), 0777);
        std::cout << baseName_path << std::endl;

        // Copy config_file into build directory
        std::ifstream cfg(argv[0], ios::binary);
        std::ofstream info_file((baseName_path + "build/info_config_file.txt").c_str(), ios::binary);
        info_file << cfg.rdbuf();
        cfg.close();
        info_file.close();

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

                std::vector<std::pair<cv::Mat, double>> output_images;
                // Process Data to create patches from this image + relevance map
                RelevanceMapProcessing(loaded_cloud, loaded_image,
                                       output_images, classifier_archive,
                                       cic_x_min , cic_x_max,
                                       cic_y_min, cic_y_max,
                                       cic_z_min, cic_z_max,
                                       cic_x_marge, cic_y_marge,
                                       imageSize_h, imageSize_w);


                // Save Data
                // Each patch will be saved under a specific image, whose name is composed of the current base image name + predicted value on a 3 number precision.
                std::cout << "Saving ... " << std::endl;
                int patch_nb = 0;
                for (std::vector<std::pair<cv::Mat, double>>::iterator it = output_images.begin() ; it != output_images.end(); ++it)
                {
                    // compute patch name
                    std::string patch_name = + "_" + std::to_string(patch_nb) + "_" + std::to_string((int(it->second * 1000)));
                    std::string filename = baseName_path + "build/_processed_" + currentFile.substr(0, currentFile.find("_")) + patch_name + "_color.jpg";
                    std::cout << " -- Saving image in: " + filename << std::endl;
                    cv::imwrite(filename, it->first);
                    patch_nb++;
                }
            }
        }

    }
    catch(std::exception& e)
    {
        std::cerr << "Unhandled Exception" << e.what() << ", application will now exit" << std::endl;
        return 2;
    }

    return 0;
}


int main (int argc, char **argv)
{
    if (argv[1] == std::string("rmp"))
    {
        std::cout << "Entering RelevanceMap Processing" << std::endl;
        LoopOverDataset_RelevanceMap(argc-2, argv + 2);
    }
    else if (argv[1] == std::string("osp"))
    {
        std::cout << "Entering ObjectSegmentation Processing" << std::endl;
        LoopOverDataset_ProcessData(argc-2, argv + 2);
    }
    else
    {
        std::cout << "This function only admits following main parameters ['rmp': RelevanceMapProcessing, 'osp': ObjectSegmentationProcessing]" << std::endl;
    }
    return 0;
}
