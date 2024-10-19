#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <iomanip>
#include <sstream>
#include <filesystem>

class PointCloudConverter : public rclcpp::Node {
public:
    PointCloudConverter() : Node("pointcloud_converter"), file_count_(0) {
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "bnk/top/points", 10, std::bind(&PointCloudConverter::pointcloud_callback, this, std::placeholders::_1));
    }

private:
    void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        pcl::PCLPointCloud2 pcl_pc2;
        pcl_conversions::toPCL(*msg, pcl_pc2);

        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromPCLPointCloud2(pcl_pc2, cloud);

        save_cloud(cloud);
    }

    void save_cloud(const pcl::PointCloud<pcl::PointXYZ>& cloud) {
        pcl::PointCloud<pcl::PointXYZ> filtered_cloud;

        for (const auto& point : cloud) {
            if (point.x != 0.0f || point.y != 0.0f || point.z != 0.0f) {
                filtered_cloud.push_back(point);
            }
        }

        std::ostringstream file_name;
        file_name << "bag/" << std::setw(4) << std::setfill('0') << ++file_count_ << ".pcd";

        std::filesystem::create_directories("bag");

        pcl::io::savePCDFileASCII(file_name.str(), filtered_cloud);
        RCLCPP_INFO(this->get_logger(), "Saved point cloud to %s", file_name.str().c_str());
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    int file_count_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudConverter>());
    rclcpp::shutdown();
    return 0;
}
