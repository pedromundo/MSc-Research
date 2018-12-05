#include <iostream>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>

using namespace std;

int main(int argc, char **argv)
{
    uint num_captures = 18;
    uint capture_step = 20;
    std::string capture_name = "cap";

    if(argc > 1){
        capture_name = std::string(argv[1]);
    }
    if(argc > 2){
        capture_step = std::stoi(std::string(argv[2]));
    }
    if(argc > 3){
        num_captures = std::stoi(std::string(argv[3]));
    }

    float theta = M_PI / 4;
    typedef pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr PointCloudWithNormals;
    typedef pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBNormal> PointCloudColorHandler;

    vector<PointCloudWithNormals> point_clouds;

    PointCloudWithNormals accumulated(new pcl::PointCloud<pcl::PointXYZRGBNormal>);

    for (size_t i = 0; i < num_captures; ++i)
    {
        point_clouds.push_back(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBNormal>));
        if (pcl::io::loadPLYFile<pcl::PointXYZRGBNormal>(string(capture_name) += (to_string(i * capture_step)) += ".ply", *point_clouds[i]) != -1)
        {
            cout << (string("MESH ") += (to_string(i * capture_step)) += " LOADED ALRIGHT!") << endl;
        }
    }

    //Translation to origin and alignment according to our rotation parameters
    for (size_t i = 0; i < num_captures; ++i)
    {
        //Cleaning
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*point_clouds[i], *point_clouds[i], indices);
        //Translating to center
        Eigen::Affine3f transform = Eigen::Affine3f::Identity();
        //650 must be the distance from kinect to the center of the table
        transform.translation() << 0.0, 0.0, 600.0;
        pcl::transformPointCloud(*point_clouds[i], *point_clouds[i], transform);
        //Rotate around center
        transform = Eigen::Affine3f::Identity();
        Eigen::Matrix3f rotation(Eigen::AngleAxisf(-DEG2RAD(i * capture_step + i * 1.17), Eigen::Vector3f::UnitY()));
        transform.rotate(rotation);
        pcl::transformPointCloud(*point_clouds[i], *point_clouds[i], transform);
    }

    //Fine alignment via ICP/ICP-NL -- both make it worse, simply rotating according
    //to our calibration works considerably better
    pcl::copyPointCloud(*point_clouds[0],*accumulated); //copy first cloud
    for (size_t i = 1; i < num_captures; ++i)
    {
        PointCloudWithNormals aligned(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
        //ICP Alignment and Registration
        // pcl::IterativeClosestPoint<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> icp;
        // icp.setInputSource(point_clouds[i]);
        // icp.setInputTarget(accumulated);
        // icp.setEuclideanFitnessEpsilon(1);
        // icp.setMaximumIterations(200);
        // icp.setTransformationEpsilon(1e-9);
        // icp.align(*aligned);
        // cout << (i*capture_step) << " has converged -- "<< " score: " << icp.getFitnessScore() << endl;
        // cout << icp.getFinalTransformation() << endl;
        // *accumulated = *accumulated + *aligned;
        *accumulated = *accumulated + *point_clouds[i]; //for tests without icp
    }

    //Visualization
    PointCloudColorHandler accumulated_color_hander(accumulated);

    pcl::visualization::PCLVisualizer viewer("Matrix transformation example");
    viewer.addPointCloud(accumulated, accumulated_color_hander, "Accumulated Cloud");
    viewer.addCoordinateSystem(1.0, "cloud", 0);
    viewer.setBackgroundColor(0, 0, 0, 0); // Setting background to black
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "Accumulated Cloud");

    //Save aligned mesh
    pcl::io::savePLYFileASCII("accumulated.ply", *accumulated);
    while (!viewer.wasStopped())
    {   // Display the visualiser until 'q' key is pressed
        viewer.spinOnce();
    }
    return (0);
}