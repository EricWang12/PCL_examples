// #include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <thread>
#include <chrono>
#include <pcl/console/parse.h>
#include <pcl/common/common_headers.h>

using namespace std::literals::chrono_literals;

int user_data;
    
void 
viewerOneOff (pcl::visualization::PCLVisualizer& viewer)
{
    viewer.setBackgroundColor (1.0, 0.5, 1.0);
    pcl::PointXYZ o;
    o.x = 1.0;
    o.y = 0;
    o.z = 0;
    viewer.addSphere (o, 0.25, "sphere", 0);
    std::cout << "i only run once" << std::endl;
    
}
    
void 
viewerPsycho (pcl::visualization::PCLVisualizer& viewer)
{
    static unsigned count = 0;
    std::stringstream ss;
    ss << "Once per viewer loop: " << count++;
    viewer.removeShape ("text", 0);
    viewer.addText (ss.str(), 200, 300, "text", 0);
    
    //FIXME: possible race condition here:
    user_data++;
}

pcl::visualization::PCLVisualizer::Ptr rgbVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}


int 
main ()
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::io::loadPCDFile ("my_point_cloud.pcd", *cloud);
    pcl::visualization::PCLVisualizer::Ptr viewer;
    viewer = rgbVis(cloud);
    // // pcl::visualization::CloudViewer viewer("Cloud Viewer");
    
    // // //blocks until the cloud is actually rendered
    // viewer.showCloud(cloud);
    
    // // //use the following functions to get access to the underlying more advanced/powerful
    // // //PCLVisualizer
    
    // // //This will only get called once
    // viewer.runOnVisualizationThreadOnce (viewerOneOff);
    
    // // //This will get called once per visualization iteration
    // viewer.runOnVisualizationThread (viewerPsycho);
    pcl::ModelCoefficients coeffs;
    //  (ax + by + cz + d = 0)
    coeffs.values.push_back(0.0);
    coeffs.values.push_back(0.0);
    coeffs.values.push_back(1.0);
    coeffs.values.push_back(0.0);
    viewer->addPlane (coeffs, "plane");
    while (!viewer->wasStopped ())
    {
    //you can also do cool processing here
    //FIXME: Note that this is running in a separate thread from viewerPsycho
    //and you should guard against race conditions yourself...
    // user_data++;
        viewer->spinOnce (100);
        std::this_thread::sleep_for(100ms);
    }
    return 0;
}