#include <iostream>

#include <pcl/io/pcd_io.h>

#include <pcl/point_types.h>

#include <pcl/filters/voxel_grid.h>

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>


boost::shared_ptr<pcl::visualization::PCLVisualizer> rgbVis (pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> single_color(cloud, 255, 10, 10);  
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, single_color, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}

unsigned int text_id = 0;
void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event,
                            void* viewer_void)
{
  pcl::visualization::PCLVisualizer *viewer = static_cast<pcl::visualization::PCLVisualizer *> (viewer_void);
  if (event.getKeySym () == "r" && event.keyDown ())
  {
    std::cout << "r was pressed => removing all text" << std::endl;

    char str[512];
    for (unsigned int i = 0; i < text_id; ++i)
    {
      sprintf (str, "text#%03d", i);
      viewer->removeShape (str);
    }
    text_id = 0;
  }
}

void mouseEventOccurred (const pcl::visualization::MouseEvent &event,
                         void* viewer_void)
{
  pcl::visualization::PCLVisualizer *viewer = static_cast<pcl::visualization::PCLVisualizer *> (viewer_void);
  if (event.getButton () == pcl::visualization::MouseEvent::LeftButton &&
      event.getType () == pcl::visualization::MouseEvent::MouseButtonRelease)
  {
    std::cout << "Left mouse button released at position (" << event.getX () << ", " << event.getY () << ")" << std::endl;

    char str[512];
    sprintf (str, "text#%03d", text_id ++);
    viewer->addText ("clicked here", event.getX (), event.getY (), str);
  }
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> interactionCustomizationVis ()
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addCoordinateSystem (1.0);

  viewer->registerKeyboardCallback (keyboardEventOccurred, (void*)viewer.get ());
  viewer->registerMouseCallback (mouseEventOccurred, (void*)viewer.get ());

  return (viewer);
}

int main (int argc, char** argv)

{

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);


  // Fill in the cloud data

  pcl::PCDReader reader;

  // Replace the path below with the path where you saved your file

  reader.read ("table_scene_lms400.pcd", *cloud); // Remember to download the file first!


  std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height 

       << " data points (" << pcl::getFieldsList (*cloud) << ")." << std::endl;


  // Create the filtering object

  pcl::VoxelGrid<pcl::PointXYZRGB> sor;

  sor.setInputCloud (cloud);

  sor.setLeafSize (0.01f, 0.01f, 0.01f);

  sor.filter (*cloud_filtered);


  std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height 

       << " data points (" << pcl::getFieldsList (*cloud_filtered) << ")." << std::endl;


  pcl::io::savePCDFileASCII ("table_scene_lms400_downsampled.pcd", *cloud_filtered);
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
  viewer = rgbVis(cloud);
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer2;
  viewer2 = rgbVis(cloud_filtered);
  /*pcl::PCDWriter writer;

  writer.write ("table_scene_lms400_downsampled.pcd", *cloud_filtered, 

         Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), false);

     */
  //--------------------
  // -----Main loop-----
  //--------------------
  while (!viewer->wasStopped ())
  {
    viewer->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }
}