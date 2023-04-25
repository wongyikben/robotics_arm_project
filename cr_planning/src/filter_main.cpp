
#include <memory>
#include "cr_planning/pcl_filter.hpp"


int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node_ptr = std::make_shared<rclcpp::Node>("pcl_filter_node");
    auto plc_filter = cr_planning::PCLFilter(node_ptr);
    rclcpp::spin(node_ptr);
    rclcpp::shutdown();
    return 0;
}