#include <iostream>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/registration/icp.h>

int main (int argc, char** argv)
{
  float theta = M_PI/4;
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_0 (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_45 (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_90 (new pcl::PointCloud<pcl::PointXYZRGBNormal>);

  //Creating color handlers
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBNormal> cloud_0_color_hander (cloud_0, 255, 0, 0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBNormal> cloud_45_color_hander (cloud_45, 0, 0, 255);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBNormal> cloud_90_color_hander (cloud_90, 0, 255, 255);

  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr aligned(new pcl::PointCloud<pcl::PointXYZRGBNormal>);

  if (pcl::io::loadPLYFile<pcl::PointXYZRGBNormal> ("dataset-estatua-sr/estatua_mesh_0.ply", *cloud_0) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    return (-1);
  }
  if (pcl::io::loadPLYFile<pcl::PointXYZRGBNormal> ("dataset-estatua-sr/estatua_mesh_45.ply", *cloud_45) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    return (-1);
  }
  if (pcl::io::loadPLYFile<pcl::PointXYZRGBNormal> ("dataset-estatua-sr/estatua_mesh_90.ply", *cloud_90) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    return (-1);
  }

  //Rotating around centroid, might be wrong
  Eigen::Affine3f transform = Eigen::Affine3f::Identity();
  Eigen::Matrix3f rotation (Eigen::AngleAxisf((45.0*M_PI) / 180, Eigen::Vector3f::UnitY()));
  Eigen::Vector4f centroid (Eigen::Vector4f::Zero());
  transform.rotate(rotation);
  pcl::compute3DCentroid(*cloud_45, centroid);
  Eigen::Vector4f centroid_new (Eigen::Vector4f::Zero());
  centroid_new.head<3>() = rotation * centroid.head<3>();
  transform.translation() = centroid.head<3>() - centroid_new.head<3>();
  pcl::transformPointCloud(*cloud_45, *cloud_45, transform);

  //ICP Alignment and Registration
  pcl::IterativeClosestPoint<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> icp;
  icp.setInputSource(cloud_45);
  icp.setInputTarget(cloud_0);
  icp.setMaximumIterations (100);
  icp.setTransformationEpsilon (1e-9);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBNormal> aligned_color_hander (aligned, 0, 255, 0);
  icp.align(*aligned);
  std::cout << "has converged:" << icp.hasConverged() << " score: " <<
  icp.getFitnessScore() << std::endl;
  std::cout << icp.getFinalTransformation() << std::endl;

  //Visualization
  pcl::visualization::PCLVisualizer viewer ("Matrix transformation example");
  viewer.addPointCloud (cloud_0, cloud_0_color_hander, "base_cloud");
  viewer.addPointCloud (aligned, aligned_color_hander, "ICP CLOUD");
  viewer.addPointCloud (cloud_45, cloud_45_color_hander, "transformed_cloud");
  viewer.addCoordinateSystem (1.0, "cloud", 0);
  viewer.setBackgroundColor(0, 0, 0, 0); // Setting background to a dark grey
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "base_cloud");
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "transformed_cloud");
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "ICP CLOUD");

  //Save aligned mesh
  pcl::io::savePLYFileASCII("teste.ply",(*cloud_0 + *aligned));
  while (!viewer.wasStopped ()) { // Display the visualiser until 'q' key is pressed
    viewer.spinOnce ();
  }

  return (0);
}