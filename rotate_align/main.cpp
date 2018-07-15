#include <iostream>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/registration/icp.h>

using namespace std;

int main(int argc, char **argv)
{
  static const uint NUM_CAPTURES = 8;
  static const uint CAPTURE_STEP = 45;

  float theta = M_PI / 4;
  typedef pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr PointCloudWithNormals;
  typedef pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBNormal> PointCloudColorHandler;

  vector<PointCloudWithNormals> point_clouds;

  PointCloudWithNormals accumulated(new pcl::PointCloud<pcl::PointXYZRGBNormal>);

  for (size_t i = 0; i < NUM_CAPTURES; ++i)
  {
    point_clouds.push_back(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBNormal>));
    if (pcl::io::loadPLYFile<pcl::PointXYZRGBNormal>(string("dataset-estatua-sr/estatua_mesh_") += (to_string(i * CAPTURE_STEP)) += ".ply", *point_clouds[i]) != -1)
    {
      cout << (string("MESH ") += (to_string(i * CAPTURE_STEP)) += " LOADED ALRIGHT!") << endl;
    }
  }\

  //Accumulated cloud starts with the first cloud
  pcl::copyPointCloud(*point_clouds[0], *accumulated);

  for (size_t i = 1; i < NUM_CAPTURES; ++i)
  {
    PointCloudWithNormals aligned(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    //Specifying the center of rotation
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    Eigen::Matrix3f rotation(Eigen::AngleAxisf((i * CAPTURE_STEP * M_PI) / 180, Eigen::Vector3f::UnitY()));
    Eigen::Vector4f centroid(Eigen::Vector4f::Zero());
    transform.rotate(rotation);
    centroid.x() = -26.721001;
    centroid.y() = -47.792292;
    centroid.z() = -620.000000; //works well, probably the center of the table
    centroid.w() = 1;
    //pcl::compute3DCentroid(*point_clouds[0], centroid);
    Eigen::Vector4f centroid_new(Eigen::Vector4f::Zero());
    centroid_new.head<3>() = rotation * centroid.head<3>();
    transform.translation() = centroid.head<3>() - centroid_new.head<3>();
    pcl::transformPointCloud(*point_clouds[i], *point_clouds[i], transform);
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
    //*accumulated = (*accumulated + *point_clouds[i]);
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
  { // Display the visualiser until 'q' key is pressed
    viewer.spinOnce();
  }
  return (0);
}