#include <memory>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

using namespace std::chrono_literals;

#define LINE std::cout << __LINE__ << std::endl

class Subscriber : public rclcpp::Node
{
  public:
    Subscriber():Node("subscriber")
    {
      this->declare_parameter<std::string>("bin_filepath", "/home/file/path/");
      this->get_parameter("bin_filepath", bin_filepath_);
      this->declare_parameter<std::string>("topic_name", "/sensing/lidar/top/rectified/pointcloud");
      this->get_parameter("topic_name", topic_name_);

      subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      topic_name_, 
      rclcpp::SensorDataQoS().keep_last(1), 
      std::bind(&Subscriber::topic_callback, this, std::placeholders::_1));
      timer_ = this->create_wall_timer(1000ms, std::bind(&Subscriber::timer_callback, this));
    }

  private:
    void topic_callback(const sensor_msgs::msg::PointCloud2 & msg)
    {
      std::string count_s = std::to_string(count);
      assert(8 > count_s.length());
      bin_file_name_ = std::string(8-count_s.length(), '0') + count_s + ".bin";
LINE;
      pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_pointcloud(new pcl::PointCloud<pcl::PointXYZI>);
      pcl::fromROSMsg(msg, *pcl_pointcloud);
      std::ofstream bin_file(bin_file_name_.c_str(), std::ios::out|std::ios::binary|std::ios::app);
      for (size_t i = 0; i < pcl_pointcloud->points.size (); ++i)
      {
        bin_file.write((char*)&pcl_pointcloud->points[i].x, 3*sizeof(float)); 
        bin_file.write((char*)&pcl_pointcloud->points[i].intensity, sizeof(float));
      }
      bin_file.close();

      count ++;
      keepdoing = true;
    }
    void timer_callback()
    {
      if(!keepdoing)
      {
        // rclcpp::shutdown();
      }
      keepdoing = false;
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;

    int count = 0;
    std::string topic_name_;
    std::string bin_file_name_;
    std::string bin_filepath_;
    bool keepdoing = true;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Subscriber>());
  rclcpp::shutdown();
  return 0;
}