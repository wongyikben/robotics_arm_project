#include <cr_planning/pcl_filter.hpp>

namespace cr_planning
{

// Compute the shorted distance between point c and line segment ab
float getLineSegment(Eigen::Vector3f a, Eigen::Vector3f b, Eigen::Vector3f c)
{
  auto u = -(a - c).dot(b - a) / (b - a).norm();
  if (u >= 0 && u <= 1) {
    return ((a - c) + u * (b - a)).norm();
  } else if ( u < 0) {
    return (c - a).norm();
  } else {
    return (c - b).norm();
  }
  return 0.0;
}

PCLFilter::PCLFilter(
  std::shared_ptr<rclcpp::Node> node)
{
  node_ = node;
  tf_buffer_ =
    std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ =
    std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  point_cloud_sub_ =
    node_->create_subscription<PointCloud2>(
      "/depth_camera/points",
      10,
      std::bind(&PCLFilter::pointCloudCb, this, std::placeholders::_1)
    );
  filtered_point_pub_ =
   node_->create_publisher<PointCloud2>(
     "/filtered/points",
     10
   );
}

TransformStamped PCLFilter::getTransform(std::string target_frame, std::string source_frame)
{
  TransformStamped t;
  try {
    t = tf_buffer_->lookupTransform(
      target_frame, source_frame,
      tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_INFO(
      node_->get_logger(), "Could not transform %s to %s: %s",
      target_frame.c_str(),
      source_frame.c_str(),
      ex.what());
    return t;
  }
  return t;
}

void PCLFilter::pointCloudCb(std::shared_ptr<PointCloud2> msg)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud =
    boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

  // This will convert the message into a pcl::PointCloud
  pcl::fromROSMsg(*msg, *cloud);
  auto t = getTransform("world", "depth_camera/link/depth_camera1");

  Eigen::Matrix4f eigen_transform;
  auto translation = t.transform.translation;
  auto rotation = t.transform.rotation;
  Eigen::Quaternion<float> rotation_quat(
    rotation.w,
    rotation.x,
    rotation.y,
    rotation.z
  );

  eigen_transform.block<3, 1>(0, 3) = Eigen::Vector3f(
    translation.x,
    translation.y,
    translation.z
  );
  eigen_transform.block<3, 3>(0, 0) = rotation_quat.toRotationMatrix();

  pcl::PointCloud<pcl::PointXYZ>::Ptr new_cloud =
    boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

  // Transform the pointclound from depth camera frame to world frame
  pcl::transformPointCloud<pcl::PointXYZ>(*cloud, *cloud, eigen_transform);

  std::vector<std::string> link_name = {
    "shoulder_link",
    "upper_arm_link",
    "forearm_link",
    "wrist_1_link",
    "wrist_3_link",
    "ee_link"
  };

  std::vector<Eigen::Vector3f> link_pos;
  for (auto & name: link_name) {
    auto link_t = getTransform("world", name);
    link_pos.push_back(
      Eigen::Vector3f(
        link_t.transform.translation.x,
        link_t.transform.translation.y,
        link_t.transform.translation.z
      )
    );
  }

  // Filter out points that are too close to the robot, otherwise it will also collide with the point
  for (int i = 0; i < static_cast<int>(cloud->points.size()); i+=10)
  {
    Eigen::Vector3f point_vec(
      cloud->points[i].x,
      cloud->points[i].y,
      cloud->points[i].z
    );
    bool isFiltered = false;
    for (int j = 1; j < static_cast<int>(link_pos.size()); j++) {
      if (getLineSegment(link_pos[j-1], link_pos[j], point_vec) < 0.25){
        isFiltered = true;
        break;
      }
    }
    if (!isFiltered) {
      new_cloud->points.push_back(cloud->points[i]);
    }
  }

  PointCloud2 filtered_msg;
  pcl::toROSMsg(*new_cloud, filtered_msg);
  filtered_msg.header.frame_id = "world";
  filtered_point_pub_->publish(filtered_msg);
}



}