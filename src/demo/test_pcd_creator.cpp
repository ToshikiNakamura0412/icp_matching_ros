#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <ros/ros.h>
#include <tf/tf.h>

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "test_pcd_creator");
  ROS_INFO_STREAM(ros::this_node::getName() << " node has started..");

  float test_2d_points[][2] = {
      {1, 2}, {3, 4}, {5, 6}, {7, 8}, {9, 10}, {1, 0}, {3, 0}, {5, 0}, {7, 0}, {9, 0},
  };

  pcl::PointCloud<pcl::PointXYZ> src_cloud;
  src_cloud.is_dense = false;
  src_cloud.resize(sizeof(test_2d_points) / sizeof(test_2d_points[0]));
  for (const auto &point : test_2d_points)
  {
    pcl::PointXYZ pcl_point;
    pcl_point.x = point[0];
    pcl_point.y = point[1];
    pcl_point.z = 0;
    src_cloud.push_back(pcl_point);
  }

  tf::Quaternion q;
  q.setRPY(0, 0, 1.0);
  tf::Transform tf(q, tf::Vector3(5.0, -2.0, 0.0));
  pcl::PointCloud<pcl::PointXYZ> target_cloud;
  pcl_ros::transformPointCloud(src_cloud, target_cloud, tf);

  pcl::io::savePCDFileASCII("src_pcd.pcd", src_cloud);
  ROS_INFO_STREAM("Saved PCD to ./src_pcd.pcd");
  pcl::io::savePCDFileASCII("target_pcd.pcd", target_cloud);
  ROS_INFO_STREAM("Saved PCD to ./target_pcd.pcd");

  return 0;
}
