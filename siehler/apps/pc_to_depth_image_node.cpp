// Pointcloud to Depth Image Node

#include <ros/ros.h>
#include <ros/package.h>

// PCL includes:
#include <pcl/conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>
#include <pcl/console/print.h>
#include <pcl/range_image/range_image.h>
#include <pcl/range_image/range_image_planar.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/transforms.h>

#include <ncurses.h>
#include <cstring>

#include <opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>

#include "sensor_msgs/Image.h"

//#include "range_image_planar_new.h"


namespace pc = pcl::console;
using namespace pcl;
using namespace std;

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef sensor_msgs::Image Image;


bool new_cloud_available_flag = false;
//PointCloudT::Ptr cloud_(new PointCloudT);
PointCloud<PointXYZ>::Ptr cloud_xyz_(new PointCloud<PointXYZ>);
PointCloud<PointWithViewpoint>::Ptr cloud_viewpoint_(new PointCloud<PointWithViewpoint>);

Image::Ptr image_(new Image);
RangeImagePlanar::Ptr range_image_(new RangeImagePlanar);
RangeImage::Ptr range_image_2(new RangeImage);

// values for sensor_pose
static float transX = -0.70, transY = -0.70, transZ = -0.35, rotX = -0.00, rotY = 0.50, rotZ = -1.50;
static float q1 = 0.0, q2 = 0.0, q3 = 0.0, q4 = 1.0;

pcl::visualization::RangeImageVisualizer viewer("Planar range image");
pcl::visualization::CloudViewer cloud_viewer_("Simple Cloud Viewer");

void
//cloud_cb (const PointCloudT::ConstPtr& callback_cloud)
cloud_cb ( const sensor_msgs::PointCloud2ConstPtr& msg )
{
  fromROSMsg(*msg, *cloud_xyz_);
  //fromROSMsg(*msg, *cloud_viewpoint_);
  
  // Transformation of PointCloud
  /*
  Eigen::Affine3f transform = Eigen::Affine3f::Identity();
  transform.translation() << 0.0, 0.0, -2.0;
  transform.rotate(Eigen::AngleAxisf((0.0*M_PI) / 180, Eigen::Vector3f::UnitX()));
  transform.rotate(Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitY()));
  transform.rotate(Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitZ()));
  pcl::transformPointCloud(*cloud_xyz_, *cloud_xyz_, transform);
  */
  cloud_viewer_.showCloud(cloud_xyz_);  
  
  new_cloud_available_flag = true;
}

void
read_kb_and_react()
{  
  static int state = 0; // 7 States needed for changing the 3 trans and 3 rot values + 1 default state  

  char c = getch();
  if(c == 'x')
  {
    state = 1;   
    printw("entered X changing state\n");
  }
  else if(c == 'y')
  {
    state = 2;
    printw("entered Y changing staten\n");
  }
  else if(c == 'z')
  {
    state = 3;
    printw("entered Z changing staten\n");
  }
  else if(c == '1')
  {
    state = 4;
    printw("entered rotX changing staten\n");
  }
  else if(c == '2')
  {
    state = 5;
    printw("entered rotY changing staten\n");
  }
  else if(c == '3')
  {
    state = 6;
    printw("entered rotZ changing state\n");
  }
  else if(c == 'u')
  {
    state = 7;
    printw("entered rotZ changing state\n");
  }
  else if(c == 'i')
  {
    state = 8;
    printw("entered rotZ changing state\n");
  }
  else if(c == 'o')
  {
    state = 9;
    printw("entered rotZ changing state\n");
  }
  else if(c == 'p')
  {
    state = 10;
    printw("entered rotZ changing state\n");
  }
  else if(c == '0')
  {
    printw("reset all values to 0\n");
    transX = -0.0, transY = -0.0, transZ = -0.0, rotX = 0.0, rotY = 0.0, rotZ = -0.0;
  }
  else if(c == '9')
  {
    printw("reset all values to current best\n");
    transX = -0.70, transY = -0.70, transZ = -0.35, rotX = -0.00, rotY = 0.50, rotZ = -1.50;
  }
  else if (c == '\033') { // if the first value is esc
    getch(); // skip the [
    float addVal = 0.0;
    switch(getch()) { // the real value
        case 'A':
            // code for arrow up
            addVal = 0.05;
            break;
        case 'B':
            // code for arrow down
            addVal = -0.05;
            break;        
    }
    if (addVal != 0.0)
    {
      switch (state)
      {
        case 1:
        transX += addVal;
        break;
        case 2:
        transY += addVal;
        break;
        case 3:
        transZ += addVal;
        break;
        case 4:
        rotX += addVal;
        break;
        case 5:
        rotY += addVal;
        break;
        case 6:
        rotZ += addVal;
        break;
        case 7:
        q1 += addVal;
        break;
        case 8:
        q2 += addVal;
        break;
        case 9:
        q3 += addVal;
        break;
        case 10:
        q4 += addVal;
        break;
       }
       clear();
       printw("New Values: X %f, Y %f, Z %f, rotX %f, rotY %f, rotZ %f\n", transX, transY, transZ, rotX, rotY, rotZ);
       printw("Add q vals: %f %f %f %f", q1,q2,q3,q4);
     }
  }
}

void
encode_32fc1(int width, int height)
{
  cv::Mat cv_image = cv::Mat(height, width, CV_32FC1, static_cast<void*>(range_image_->getRangesArray()), (4 * width));
  
  image_ = cv_bridge::CvImage(std_msgs::Header(), "32FC1", cv_image).toImageMsg();
  image_->height = height;
  image_->width = width;
  image_->header.stamp = ros::Time::now();
  image_->header.frame_id = "/camera_rgb_optical_frame";
}

void
encode_16uc1(int width, int height)
{
  cv::Mat cv_image = cv::Mat(height, width, CV_32FC1, static_cast<void*>(range_image_->getRangesArray()), (4 * width));
  cv_image.convertTo(cv_image,CV_16UC1);
  
  image_ = cv_bridge::CvImage(std_msgs::Header(), "16UC1", cv_image).toImageMsg();
  image_->height = height;
  image_->width = width;
  image_->header.stamp = ros::Time::now();
  image_->header.frame_id = "/camera_rgb_optical_frame"; 
}

// only works with organized PointCloud. But good results.
void
transform2()
{
  float centre_x = 640.0f / 2.0f;
  float centre_y = 480.0f / 2.0f;
  float focal_x = 525.0f;
  float focal_y = 525.0f;
  int width = 640, height = 480;

  cv::Mat cv_image = cv::Mat(height, width, CV_32FC1,cv::Scalar(std::numeric_limits<float>::max()));

  for (int i=0; i<cloud_xyz_->points.size();i++){
    if (cloud_xyz_->points[i].z == cloud_xyz_->points[i].z){
        float z = cloud_xyz_->points[i].z*1000.0;
        float u = (cloud_xyz_->points[i].x*1000.0*focal_x) / z;
        float v = (cloud_xyz_->points[i].y*1000.0*focal_y) / z;
        int pixel_pos_x = (int)(u + centre_x);
        int pixel_pos_y = (int)(v + centre_y);

    if (pixel_pos_x > (width-1)){
      pixel_pos_x = width -1;
    }
    if (pixel_pos_y > (height-1)){
      pixel_pos_y = height-1;
    }
    cv_image.at<float>(pixel_pos_y,pixel_pos_x) = z;
    }       
  }

  cv_image.convertTo(cv_image,CV_16UC1);

  image_ = cv_bridge::CvImage(std_msgs::Header(), "16UC1", cv_image).toImageMsg();
  image_->height = height;
  image_->width = width;
  image_->header.stamp = ros::Time::now();
  image_->header.frame_id = "/camera_rgb_optical_frame";
}

void
transform()
{
  read_kb_and_react();

  // FIRST create RangeImagePlanar, from: http://robotica.unileon.es/index.php/PCL/OpenNI_tutorial_4:_3D_object_recognition_(descriptors)
  int width = 640;
  int height = 480;
  int size = width * height;
  // Center of projection. here, we choose the middle of the image.
  float centerX = (float) width / 2.0f;
  float centerY = (float) height / 2.0f;
  float focalLengthX = 525.0f, focalLengthY = focalLengthX;

  Eigen::Matrix3f m;
  m = Eigen::AngleAxisf(rotX * M_PI, Eigen::Vector3f::UnitY()) * Eigen::AngleAxisf(rotY * M_PI,  Eigen::Vector3f::UnitY()) * Eigen::AngleAxisf(rotZ * M_PI, Eigen::Vector3f::UnitZ());

  Eigen::AngleAxisf rollAngle(rotX, Eigen::Vector3f::UnitZ());
  Eigen::AngleAxisf yawAngle(rotZ, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf pitchAngle(rotY, Eigen::Vector3f::UnitX());
  Eigen::Quaternionf q_2 = rollAngle * yawAngle * pitchAngle;
  Eigen::Affine3f id = Eigen::Affine3f::Identity();
  
  Eigen::Quaternionf q(m);
  Eigen::Affine3f sensorPose = Eigen::Affine3f(Eigen::Translation3f(transX, transY, transZ)) * q;
  //Eigen::Affine3f sensorPose = id * q;
  //Eigen::Affine3f sensorPose = Eigen::Affine3f(Eigen::Translation3f(transX, transY, transZ)) * q * Eigen::Quaternionf(q1, q2, q3, q4);
  //Eigen::Affine3f sensorPose = id;
  
  // Noise level. If greater than 0, values of neighboring points will be averaged.
  // This would set the search radius (e.g., 0.03 == 3cm).
  float noiseLevel = 0.0f;
  float minimumRange = 0.0f;
  int borderSize = 1;
  
  range_image_->createFromPointCloudWithFixedSize(*cloud_xyz_, width, height, centerX, centerY, focalLengthX, focalLengthY, sensorPose, pcl::RangeImage::CAMERA_FRAME, noiseLevel, minimumRange);  

  //range_image_cool_->createFromOrganizedPointCloud(*cloud_xyz_);

  //viewer.showRangeImage(*range_image_);

  // SECOND transform RangeImagePlanar to sensor_msgs/Image
  encode_32fc1(width, height);
  //encode_16uc1(width, height);
}

int
main (int argc, char** argv)
{
  // init
  ros::init(argc, argv, "pc_to_depth_image_node");
  ros::NodeHandle nh("~");

  // init console read with ncurse https://stackoverflow.com/questions/29335758/using-kbhit-and-getch-on-linux
  initscr();
  cbreak();
  noecho();
  scrollok(stdscr, TRUE);
  nodelay(stdscr, TRUE);

  // Topic names
  std::string pointcloud_topic = "/camera/depth_registered/points";
  pc::parse_argument (argc, argv, "-pointcloud", pointcloud_topic);
  std::string depth_topic = "/siehler/depth/image";
  pc::parse_argument (argc, argv, "-depth", depth_topic);  

  // Subscribers:
  ros::Subscriber sub = nh.subscribe(pointcloud_topic, 1, cloud_cb);

  // Publishers:
  ros::Publisher depth_pub;
  depth_pub= nh.advertise<Image>(depth_topic, 3);


  while (ros::ok())
  {
    if (new_cloud_available_flag)
    {
      transform();
      new_cloud_available_flag = false;
      depth_pub.publish(image_);
    }    

    ros::spinOnce();
  }
}
