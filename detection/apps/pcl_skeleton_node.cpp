// ROS includes:
#include <ros/ros.h>
#include <ros/package.h>

// PCL includes:
#include <pcl/conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>
#include <pcl/console/print.h>
#include <pcl/gpu/containers/initialization.h>
#include <pcl/gpu/people/people_detector.h>
#include <pcl/gpu/people/colormap.h>
#include <pcl/visualization/image_viewer.h>
#include <pcl/filters/passthrough.h>

#include "std_msgs/Float32MultiArray.h"
#include "sensor_msgs/Image.h"


namespace pc = pcl::console;
using namespace pcl::visualization;
using namespace pcl::gpu;
using namespace pcl::gpu::people;
using namespace pcl;
using namespace std;


typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

bool new_cloud_available_flag = false;
PointCloudT::Ptr cloud(new PointCloudT);
pcl::PointCloud<PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<PointXYZ>); 
PeopleDetector::Depth depth;
PeopleDetector::Image image_;

void
cloud_cb (const PointCloudT::ConstPtr& callback_cloud)
{
  pcl::copyPointCloud(*callback_cloud, *cloud);
  pcl::copyPointCloud(*callback_cloud, *cloud_xyz);
  new_cloud_available_flag = true;
}

void
depth_cb (const sensor_msgs::Image image)
{
  if (!image.data.empty())
  {
    depth.upload((void*)&image.data[0], image.step, (int)image.height, (int)image.width);
  }
}

void
image_cb (const sensor_msgs::Image im)
{
  if (!im.data.empty())
  {
    //image_.upload((void*)&im.data[0], im.step, (int)im.height, (int)im.width);
  }
}

void
execute_passthrough(PointCloudT::Ptr cloud, PointCloudT::Ptr filtered, const std::string field_name, float min, float max)
{
  pcl::PassThrough<PointT> pass;
  pass.setInputCloud (cloud);
  pass.setFilterFieldName (field_name);
  pass.setFilterLimits (min, max);
  pass.filter (*filtered);
}


int
main (int argc, char** argv)
{
  // init
  ros::init(argc, argv, "pcl_skeleton_detector");
  ros::NodeHandle nh("~");

  // selecting GPU and prining info
  int device = 0;
  pc::parse_argument (argc, argv, "-gpu", device);
  pcl::gpu::setDevice (device);
  pcl::gpu::printShortCudaDeviceInfo (device);

  //selecting tree files
  vector<string> tree_files;
  tree_files.push_back("/root/workspace/ros/data/1tree_20.txt");
  tree_files.push_back("/root/workspace/ros/data/2tree_20.txt");
  tree_files.push_back("/root/workspace/ros/data/3tree_20.txt");

  pc::parse_argument (argc, argv, "-tree0", tree_files[0]);
  pc::parse_argument (argc, argv, "-tree1", tree_files[1]);
  pc::parse_argument (argc, argv, "-tree2", tree_files[2]);
  pc::parse_argument (argc, argv, "-tree3", tree_files[2]);

  int num_trees = (int)tree_files.size();
  pc::parse_argument (argc, argv, "-numTrees", num_trees);

  tree_files.resize(num_trees);
  if (num_trees == 0 || num_trees > 4)
    return cout << "Invalid number of trees" << endl, -1;

  // PCL People Detector
  pcl::gpu::people::PeopleDetector people_detector;
  typedef pcl::gpu::people::RDFBodyPartsDetector RDFBodyPartsDetector;
  RDFBodyPartsDetector::Ptr rdf(new RDFBodyPartsDetector(tree_files));
  people_detector.rdf_detector_ = rdf;
  pcl::gpu::people::PeopleDetector::Image cmap_device_;
  pcl::PointCloud<pcl::RGB> cmap_host_;
  DeviceArray<pcl::RGB> color_map_;
  ImageViewer final_view_;
  cmap_device_.create(640, 480);

  depth.create(640, 480);
  //image_.create(640, 480);
  people::uploadColorMap(color_map_);

  // Topic names
  std::string pointcloud_topic = "/camera/depth_registered/points";
  pc::parse_argument (argc, argv, "-pointcloud", pointcloud_topic);
  std::string output_topic = "skeleton_detections";
  std::string depth_topic = "/camera/depth_registered/image";
  pc::parse_argument (argc, argv, "-depth", depth_topic);
  std::string image_topic = "/camera/rgb/image_raw";
  pc::parse_argument (argc, argv, "-image", image_topic);

  // Subscribers:
  ros::Subscriber sub = nh.subscribe(pointcloud_topic, 1, cloud_cb);
  ros::Subscriber sub_depth = nh.subscribe(depth_topic, 1, depth_cb);
  ros::Subscriber sub_image = nh.subscribe(image_topic, 1, image_cb);

  // Publishers:
  ros::Publisher detection_pub;
  detection_pub= nh.advertise<std_msgs::Float32MultiArray>(output_topic, 3);


  while (ros::ok())
  {
    if (new_cloud_available_flag)
    {
      //PointCloudT::Ptr filtered_cloud(new PointCloudT);
      //execute_passthrough(cloud, filtered_cloud, "z", 0.1, 3.0);

      int process_return = people_detector.process(cloud);
      //int process_return = people_detector.process(depth, image_);
      // using pcl::gpu::people::RDFBodyPartsDetector::Labels = DeviceArray2D<unsigned char>

      if (process_return == 2)
      {
        const pcl::gpu::people::PeopleDetector::Labels& labels = people_detector.rdf_detector_->getLabels();
      
        /*rdf->process(depth, *cloud_xyz, 200);
        const pcl::gpu::people::PeopleDetector::Labels& labels = rdf->getLabels();*/

        pcl::PointCloud<unsigned char> cl(labels.cols(), labels.rows());
        int cols;
        labels.download(cl.points, cols);
        for (int i=0; i < cl.points.size(); ++i){
          //std::cout << (int)cl.points[i] << " ";
        }

        people::colorizeLabels(color_map_, labels, cmap_device_);

        int c;
        cmap_host_.width = cmap_device_.cols();
        cmap_host_.height = cmap_device_.rows();
        cmap_host_.points.resize(cmap_host_.width * cmap_host_.height);
        cmap_device_.download(cmap_host_.points, c);

        final_view_.showRGBImage<pcl::RGB>(cmap_host_);
        final_view_.spinOnce(1, true);
      }      

      new_cloud_available_flag = false;
    }
    std_msgs::Float32MultiArray array;

    //TODO: map labels to a publishable data format (i.e. skeleton points)

    // publish result
    detection_pub.publish(array);

    ros::spinOnce();
  }
}
