#include <string>
#include <ros/ros.h>
#include <segmatch/database.hpp>
#include <segmatch/local_map.hpp>
#include <segmatch/segmatch.hpp>
#include <segmatch/common.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/random_sample.h>
#include <pcl_ros/io/pcd_io.h>
#include <visualization_msgs/Marker.h>

struct Color {
  Color(float red, float green, float blue) : r(red), g(green), b(blue) {}
  float r;
  float g;
  float b;
};

void convert_to_point_cloud_2_msg(const segmatch::PointICloud& cloud, const std::string& frame, sensor_msgs::PointCloud2* converted);

segmatch::SegMatchParams getSegMatchParams(const ros::NodeHandle& nh, const std::string& prefix);

void publishLineSet(const segmatch::PointPairs& point_pairs, const std::string& frame, const float line_scale, const Color& color, const ros::Publisher& publisher);