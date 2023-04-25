#ifndef CR_PLANNING__PCL_FILTER_HPP_
#define CR_PLANNING__PCL_FILTER_HPP_

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Transform.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>


using PointCloud2 = sensor_msgs::msg::PointCloud2;
using TransformStamped=geometry_msgs::msg::TransformStamped;
using Vector3=geometry_msgs::msg::Vector3;

namespace cr_planning
{
  class PCLFilter
  {
    public:
      explicit PCLFilter(std::shared_ptr<rclcpp::Node> node);
      virtual ~PCLFilter() {}

    private:
      std::shared_ptr<rclcpp::Node> node_;
      std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
      std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

      rclcpp::Publisher<PointCloud2>::SharedPtr filtered_point_pub_;
      rclcpp::Subscription<PointCloud2>::SharedPtr point_cloud_sub_;

      void pointCloudCb(std::shared_ptr<PointCloud2> msg);
      TransformStamped getTransform(std::string to_frame, std::string from_frame);

  };

}


#endif
