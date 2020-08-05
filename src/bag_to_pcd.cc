#include <ros/ros.h>

#include <pcl/io/pcd_io.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

std::string output_dir;

// cloud trandform parameters
double trans_roll_  = -0.6;
double trans_pitch_ = 0.0;
double trans_yaw_   = -90;
double trans_tx_    = 0.75;
double trans_ty_    = 0.0;
double trans_tz_    = 0.0;

Eigen::Affine3f transform_matrix_ = Eigen::Affine3f::Identity();
ros::Publisher pub_cloud_;

void PointCloudCallback(
    const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& cloud_ptr) {

  std::cerr << "Got " << cloud_ptr->size() << " data points in frame "
            << cloud_ptr->header.frame_id
            << " with the following fields: " << pcl::getFieldsList(*cloud_ptr)
            << std::endl;

  pcl::PointCloud<pcl::PointXYZI>::Ptr trans_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::transformPointCloud(*cloud_ptr, *trans_cloud_ptr,transform_matrix_);

  // debug
    sensor_msgs::PointCloud2 out_cloud_msg;
    pcl::toROSMsg(*trans_cloud_ptr, out_cloud_msg);
    //out_cloud_msg.header = cloud_ptr->header;
    pub_cloud_.publish(out_cloud_msg);

  // generate pcd file
  std::stringstream ss;
  ss << output_dir << "/" << cloud_ptr->header.stamp << ".pcd";
  pcl::io::savePCDFileASCII(ss.str(), *trans_cloud_ptr);
  std::cerr << "Data saved to " << ss.str () << std::endl;
}

int main(int argc, char** argv) {

  ros::init(argc, argv, "bag_to_pcd");

  transform_matrix_.translation() << trans_tx_, trans_ty_, trans_tz_;
  transform_matrix_.rotate(Eigen::AngleAxisf(trans_yaw_ * M_PI / 180, Eigen::Vector3f::UnitZ()));
  transform_matrix_.rotate(Eigen::AngleAxisf(trans_pitch_ * M_PI / 180, Eigen::Vector3f::UnitY()));
  transform_matrix_.rotate(Eigen::AngleAxisf(trans_roll_ * M_PI / 180, Eigen::Vector3f::UnitX()));

  // check for params
  if (argc < 3) {
    std::cerr << "Syntax is: " << argv[0] << " <cloud_topic> <output_directory>"
              << std::endl;
    std::cerr << "Example: " << argv[0] << " /pandar_points ./pcd_files"
              << std::endl;
    return (-1);
  }
  output_dir = argv[2];

  ros::NodeHandle nh;
  ros::Subscriber sub_cloud = nh.subscribe(argv[1], 10, PointCloudCallback);
  pub_cloud_ = nh.advertise<sensor_msgs::PointCloud2>("/debug_points", 2);

  ros::spin();
  return 0;
}
