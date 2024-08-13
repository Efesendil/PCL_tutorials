#include <iostream>

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>

//First goal is to visualize a point cloud in 2 viewports : Left modified - Right original
// Two will be in custom color
// Modify either by filtering, downsampling, segmentation etc...

boost::shared_ptr<pcl::visualization::PCLVisualizer> PassThrougVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{ 
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> viewportsVisNormals (
    pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, pcl::PointCloud<pcl::Normal>::ConstPtr normals1, pcl::PointCloud<pcl::Normal>::ConstPtr normals2)
{
  // --------------------------------------------------------
  // -----Open 3D viewer and add point cloud and normals-----
  // --------------------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->initCameraParameters ();

  int v1(0);
  viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
  viewer->setBackgroundColor (0, 0, 0, v1);
  viewer->addText("Radius: 0.01", 10, 10, "v1 text", v1);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> single_color1(cloud, 255, 0, 0);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, single_color1, "sample cloud1", v1);

  int v2(0);
  viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
  viewer->setBackgroundColor (0.3, 0.3, 0.3, v2);
  viewer->addText("Radius: 0.1", 10, 10, "v2 text", v2);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> single_color2(cloud, 0, 255, 0);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, single_color2, "sample cloud2", v2);

  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud1");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud2");
  viewer->addCoordinateSystem (1.0);

  viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (cloud, normals1, 10, 0.05, "normals1", v1);
  viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (cloud, normals2, 10, 0.05, "normals2", v2);

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

int main(){

    // ------------------------------------
  // -----Create example point cloud-----
  // ------------------------------------
  pcl::PointCloud<pcl::PointXYZ>::Ptr basic_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
  std::cout << "Generating example point clouds.\n\n";
  // We're going to make an ellipse extruded along the z-axis. The colour for
  // the XYZRGB cloud will gradually go from red to green to blue.
  uint8_t r(255), g(15), b(15);
  for (float z(-1.0); z <= 1.0; z += 0.05)
  {
    for (float angle(0.0); angle <= 360.0; angle += 5.0)
    {
      pcl::PointXYZ basic_point;
      basic_point.x = 0.5 * cosf (pcl::deg2rad(angle));
      basic_point.y = sinf (pcl::deg2rad(angle));
      basic_point.z = z;
      basic_cloud_ptr->points.push_back(basic_point);

      pcl::PointXYZRGB point;
      point.x = basic_point.x;
      point.y = basic_point.y;
      point.z = basic_point.z;
      uint32_t rgb = (static_cast<uint32_t>(r) << 16 |
              static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
      point.rgb = *reinterpret_cast<float*>(&rgb);
      point_cloud_ptr->points.push_back (point);
    }
    if (z < 0.0)
    {
      r -= 12;
      g += 12;
    }
    else
    {
      g -= 12;
      b += 12;
    }
  }
  basic_cloud_ptr->width = (int) basic_cloud_ptr->points.size ();
  basic_cloud_ptr->height = 1;
  point_cloud_ptr->width = (int) point_cloud_ptr->points.size ();
  point_cloud_ptr->height = 1;

  // ----------------------------------------------------------------
  // -----Calculate surface normals with a search radius of 0.05-----
  // ----------------------------------------------------------------
  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
  ne.setInputCloud (point_cloud_ptr);
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
  ne.setSearchMethod (tree);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals1 (new pcl::PointCloud<pcl::Normal>);
  ne.setRadiusSearch (0.05);
  ne.compute (*cloud_normals1);

  // ---------------------------------------------------------------
  // -----Calculate surface normals with a search radius of 0.1-----
  // ---------------------------------------------------------------
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2 (new pcl::PointCloud<pcl::Normal>);
  ne.setRadiusSearch (0.1);
  ne.compute (*cloud_normals2);


  std::cerr << "Cloud before filtering: " << std::endl;
    for (size_t i = 0; i < basic_cloud_ptr->points.size (); ++i)
      std::cerr << "    " << basic_cloud_ptr->points[i].x << " " 
                          << basic_cloud_ptr->points[i].y << " " 
                          << basic_cloud_ptr->points[i].z << std::endl;
  // ---------------------------------------------------------------
  // -----Create the pass through filter filtering object-----------
  // ---------------------------------------------------------------
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (basic_cloud_ptr);
  pass.setFilterFieldName ("y");
  pass.setFilterLimits (0.0, 0.5);
  pass.setFilterFieldName("x");
  pass.setFilterLimits (0.0, 0.5);
  pass.setFilterFieldName("z");
  pass.setFilterLimits (0.0, 0.5);
  //Works only for one field name
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  //pass.setFilterLimitsNegative (true);
  pass.filter (*cloud_filtered);
  // ---------------------------------------------------------------
  std::cerr << "Cloud after filtering: " << std::endl;
  for (size_t i = 0; i < cloud_filtered->points.size (); ++i)
    std::cerr << "    " << cloud_filtered->points[i].x << " " 
                        << cloud_filtered->points[i].y << " " 
                        << cloud_filtered->points[i].z << std::endl;



  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
  //viewer = viewportsVisNormals(point_cloud_ptr, cloud_normals1, cloud_normals2);
  viewer = PassThrougVis(cloud_filtered);

    //--------------------
  // -----Main loop-----
  //--------------------
  while (!viewer->wasStopped ())
  {
    viewer->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }
}