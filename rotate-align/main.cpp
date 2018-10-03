#include <iostream>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/registration/icp_nl.h>

using namespace std;

int main(int argc, char **argv)
{
    static const uint NUM_CAPTURES = 18;
    static const uint CAPTURE_STEP = 20;

    float theta = M_PI / 4;
    typedef pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr PointCloudWithNormals;
    typedef pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBNormal> PointCloudColorHandler;

    vector<PointCloudWithNormals> point_clouds;

    PointCloudWithNormals accumulated(new pcl::PointCloud<pcl::PointXYZRGBNormal>);

    for (size_t i = 0; i < NUM_CAPTURES; ++i)
    {
        point_clouds.push_back(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBNormal>));
        if (pcl::io::loadPLYFile<pcl::PointXYZRGBNormal>(string("/home/pedroraimundo/Desktop/tartaruga_metodo_sr/tartaruga_srmesh_") += (to_string(i * CAPTURE_STEP)) += ".ply", *point_clouds[i]) != -1)
        {
            cout << (string("MESH ") += (to_string(i * CAPTURE_STEP)) += " LOADED ALRIGHT!") << endl;
        }
    }

    //Translation to origin and rough alignment
    for (size_t i = 0; i < NUM_CAPTURES; ++i)
    {
        //Translating to center
        Eigen::Affine3f transform = Eigen::Affine3f::Identity();
        transform.translation() << 0.0, 0.0, 650.0; //650 must be the distance from kinec to the center of the table
        pcl::transformPointCloud(*point_clouds[i], *point_clouds[i], transform);
        //Rotate around center
        transform = Eigen::Affine3f::Identity();
        Eigen::Matrix3f rotation(Eigen::AngleAxisf(-DEG2RAD(i * CAPTURE_STEP + i * 1.17), Eigen::Vector3f::UnitY()));
        transform.rotate(rotation);
        pcl::transformPointCloud(*point_clouds[i], *point_clouds[i], transform);
    }

    //Fine alignment via ICP/ICP-NL -- both make it worse
    //Simply rotating according to our calibration works
    //considerably better
    pcl::copyPointCloud(*point_clouds[0],*accumulated); //copy first cloud
    for (size_t i = 1; i < NUM_CAPTURES; ++i)
    {
        PointCloudWithNormals aligned(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
        //ICP Alignment and Registration
        pcl::IterativeClosestPoint<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> icp;
        icp.setInputSource(point_clouds[i]);
        icp.setInputTarget(accumulated);
        icp.setMaximumIterations(200);
        icp.setTransformationEpsilon(1e-9);
        icp.align(*aligned);
        cout << (i*CAPTURE_STEP) << " has converged -- "<< " score: " << icp.getFitnessScore() << endl;
        cout << icp.getFinalTransformation() << endl;
        *accumulated = *accumulated + *aligned;
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