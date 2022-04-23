#include <cstdlib>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"

namespace my_package
{
class Pointcloud2Talker : public rclcpp::Node
{
public:
  Pointcloud2Talker(const rclcpp::NodeOptions & options)
  : Node("pointcloud2_talker", options)
  {
    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("my_pointcloud2", 10);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&Pointcloud2Talker::timer_callback, this));
  }

private:

  void make_pc2_msg(const float object_list[], int32_t width, int32_t height, sensor_msgs::msg::PointCloud2 & msg)
  {
    /*** sample ***/
    // static const float object_list[] = {
    //   0, 0, 0,
    //   0, 0, 1,
    //   0, 1, 0,
    //   0, 1, 1,
    //   1, 0, 0,
    //   1, 0, 1,
    //   1, 1, 0,
    //   1, 1, 1,
    // };

    static const size_t POINT_CLOUD_POINT_SIZE = 3;
    static const size_t POINT_CLOUD_ELEM_SIZE = sizeof(float);

    msg.header.frame_id = "map";
    msg.header.stamp = this->get_clock()->now();
    msg.height = height;
    msg.width = width;
    msg.fields.resize(3);
    msg.fields[0].set__name("x");
    msg.fields[0].set__datatype(sensor_msgs::msg::PointField::FLOAT32);
    msg.fields[0].set__offset(0);
    msg.fields[0].set__count(1);
    msg.fields[1].set__name("y");
    msg.fields[1].set__datatype(sensor_msgs::msg::PointField::FLOAT32);
    msg.fields[1].set__offset(4);
    msg.fields[1].set__count(1);
    msg.fields[2].set__name("z");
    msg.fields[2].set__datatype(sensor_msgs::msg::PointField::FLOAT32);
    msg.fields[2].set__offset(8);
    msg.fields[2].set__count(1);

    msg.is_bigendian = false;
    msg.point_step = POINT_CLOUD_POINT_SIZE * POINT_CLOUD_ELEM_SIZE;
    msg.row_step = msg.point_step * msg.width;
    msg.data.resize(msg.row_step * msg.height);

    for (size_t y = 0; y < msg.height; y++) {
      for (size_t x = 0; x < msg.width; x++) {
        size_t offset_src = y * (msg.width * 3) + x * 3;
        const float * src_xyz = &object_list[offset_src];
        size_t offset_dst = (y * (msg.width) + x) * POINT_CLOUD_POINT_SIZE * POINT_CLOUD_ELEM_SIZE;
        float * dst = (float*)(&msg.data[offset_dst]);
        dst[0] = src_xyz[0];
        dst[1] = src_xyz[1];
        dst[2] = src_xyz[2];
      }
    }

    msg.is_dense = false;
  }

  void make_pc2_color_msg(const float object_list[], const uint8_t color_list[], int32_t width, int32_t height, sensor_msgs::msg::PointCloud2 & msg)
  {
    /*** sample ***/
    // static const float object_list[] = {
    //   0, 0, 0,
    //   0, 0, 1,
    //   0, 1, 0,
    //   0, 1, 1,
    //   1, 0, 0,
    //   1, 0, 1,
    //   1, 1, 0,
    //   1, 1, 1,
    // };
    // static const uint8_t color_list[] = {
    //   0, 0, 0,
    //   0, 0, 255,
    //   0, 255, 0,
    //   0, 255, 255,
    //   255, 0, 0,
    //   255, 0, 255,
    //   255, 255, 0,
    //   255, 255, 255,
    // };

    static const size_t POINT_CLOUD_POINT_SIZE = 4;
    static const size_t POINT_CLOUD_ELEM_SIZE = sizeof(float);

    msg.header.frame_id = "map";
    msg.header.stamp = this->get_clock()->now();
    msg.height = height;
    msg.width = width;
    msg.fields.resize(4);
    msg.fields[0].set__name("x");
    msg.fields[0].set__datatype(sensor_msgs::msg::PointField::FLOAT32);
    msg.fields[0].set__offset(0);
    msg.fields[0].set__count(1);
    msg.fields[1].set__name("y");
    msg.fields[1].set__datatype(sensor_msgs::msg::PointField::FLOAT32);
    msg.fields[1].set__offset(4);
    msg.fields[1].set__count(1);
    msg.fields[2].set__name("z");
    msg.fields[2].set__datatype(sensor_msgs::msg::PointField::FLOAT32);
    msg.fields[2].set__offset(8);
    msg.fields[2].set__count(1);
    msg.fields[3].set__name("rgb");
    msg.fields[3].set__datatype(sensor_msgs::msg::PointField::FLOAT32);
    msg.fields[3].set__offset(12);
    msg.fields[3].set__count(1);

    msg.is_bigendian = false;
    msg.point_step = POINT_CLOUD_POINT_SIZE * POINT_CLOUD_ELEM_SIZE;
    msg.row_step = msg.point_step * msg.width;
    msg.data.resize(msg.row_step * msg.height);

    for (size_t y = 0; y < msg.height; y++) {
      for (size_t x = 0; x < msg.width; x++) {
        size_t offset_src = y * (msg.width * 3) + x * 3;
        const float * src_xyz = &object_list[offset_src];
        const uint8_t * src_rgb = &color_list[offset_src];
        size_t offset_dst = (y * (msg.width) + x) * POINT_CLOUD_POINT_SIZE * POINT_CLOUD_ELEM_SIZE;
        float * dst = (float*)(&msg.data[offset_dst]);
        dst[0] = src_xyz[0];
        dst[1] = src_xyz[1];
        dst[2] = src_xyz[2];
        uint8_t r = src_rgb[0];
        uint8_t g = src_rgb[1];
        uint8_t b = src_rgb[2];
        dst[3] =  (b << 16) | (g << 8) | r;
      }
    }

    msg.is_dense = false;
  }

  void timer_callback()
  {
    static const float object_list[] = {
      0, 0, 0,
      0, 0, 1,
      0, 1, 0,
      0, 1, 1,
      1, 0, 0,
      1, 0, 1,
      1, 1, 0,
      1, 1, 1,
    };
    static const uint8_t color_list[] = {
      0, 0, 0,
      0, 0, 255,
      0, 255, 0,
      0, 255, 255,
      255, 0, 0,
      255, 0, 255,
      255, 255, 0,
      255, 255, 255,
    };

    auto msg = sensor_msgs::msg::PointCloud2();
    make_pc2_color_msg(object_list, color_list, 2, 4, msg);

    publisher_->publish(msg);
  }

private:
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  int count_;
};
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(my_package::Pointcloud2Talker)
