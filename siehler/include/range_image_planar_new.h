#include <pcl/range_image/range_image_planar.h>
#include <iostream>

namespace pcl
{

class RangeImagePlanarNew : public pcl::RangeImagePlanar
{
public:
  template <typename PointCloudType>
  void createFromOrganizedPointCloud(const PointCloudType cloud, const Eigen::Affine3f& sensor_pose = Eigen::Affine3f::Identity())
{
  if(cloud.height <= 1)
    {
    throw pcl::UnorganizedPointCloudException("Must have an organized point cloud to convert to a RangeImagePlanar using createFromOrganizedPointCloud!");
    }

  // Compute the sensor position
  Eigen::Vector3f zero;
  zero[0] = 0; zero[1] = 0; zero[2] = 0;
  Eigen::Vector3f origin = sensor_pose * zero;

  // Copy the points into the range image, and compute their range
  // (distance to the sensor)
  this->resize (cloud.size());
  for(unsigned int row = 0; row < cloud.height; ++row)
  {
    for(unsigned int col = 0; col < cloud.width; ++col)
    {
      typename PointCloudType::PointType p = cloud(col,row);
      PointWithRange range_point;
      range_point.getVector3fMap() = p.getVector3fMap();

      Eigen::Vector3f v = p.getVector3fMap() - origin;
      range_point.range = v.norm();
      (*this)(col,row) = range_point;
    }
  }
}

private:
  template <typename TPoint>
  bool IsNan(const TPoint& p)
{
  if(pcl_isnan(p.x) ||
     pcl_isnan(p.y) ||
     pcl_isnan(p.z))
  {
    return true;
  }

  return false;
}
};
}
