// wall_filter_node.cpp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <kurome/msg/rectangle.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <Eigen/Dense>
#include <cmath>
#include <vector>
#include <algorithm>

using std::placeholders::_1;

class WallFilterNode : public rclcpp::Node {
public:
    WallFilterNode() : Node("wall_filter_node") {
        this->declare_parameter<std::string>("target_frame", "map");
        this->declare_parameter<std::string>("source_frame", "base_link");

        get_parameter("target_frame", target_frame_);
        get_parameter("source_frame", source_frame_);

        default_bad_range_ = 0.0f;
        float default_arena_size = 100000.0f;
        min_x_ = -default_arena_size;
        min_y_ = -default_arena_size;
        max_x_ = default_arena_size;
        max_y_ = default_arena_size;

        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", rclcpp::SensorDataQoS(), std::bind(&WallFilterNode::laser_callback, this, _1));
        kurome_scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/kurome/scan", rclcpp::SensorDataQoS(), std::bind(&WallFilterNode::laser_callback, this, _1));
        scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/filteredLasers999", 10);

        arena_sub_ = this->create_subscription<kurome::msg::Rectangle>("/kurome/arena", 10, std::bind(&WallFilterNode::arena_callback, this, _1));

        cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("/unilidar/cloud", rclcpp::SensorDataQoS(), std::bind(&WallFilterNode::pointcloud_callback, this, _1));
        cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/testPointCloud999", 10);

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    }

private:
    void arena_callback(const kurome::msg::Rectangle::SharedPtr msg) {
        min_x_ = msg->arena.origin.x;
        min_y_ = msg->arena.origin.y;
        max_x_ = min_x_ + msg->arena.width;
        max_y_ = min_y_ + msg->arena.height;
    }

    void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        geometry_msgs::msg::TransformStamped transform;
        try {
            transform = tf_buffer_->lookupTransform(target_frame_, msg->header.frame_id, tf2::TimePointZero);
        } catch (const tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s", ex.what());
            return;
        }

        sensor_msgs::msg::PointCloud2 transformed_cloud;
        tf2::doTransform(*msg, transformed_cloud, transform);

        std::vector<uint8_t> valid_data;
        valid_data.reserve(transformed_cloud.data.size());

        for (size_t i = 0; i < transformed_cloud.data.size(); i += transformed_cloud.point_step) {
            float x = *reinterpret_cast<const float*>(&transformed_cloud.data[i + transformed_cloud.fields[0].offset]);
            float y = *reinterpret_cast<const float*>(&transformed_cloud.data[i + transformed_cloud.fields[1].offset]);

            if (x >= min_x_ && x <= max_x_ && y >= min_y_ && y <= max_y_) {
                valid_data.insert(valid_data.end(),
                                  transformed_cloud.data.begin() + i,
                                  transformed_cloud.data.begin() + i + transformed_cloud.point_step);
            }
        }

        transformed_cloud.data = valid_data;
        transformed_cloud.row_step = static_cast<uint32_t>(valid_data.size());
        transformed_cloud.width = transformed_cloud.row_step / transformed_cloud.point_step;
        cloud_pub_->publish(transformed_cloud);
    }

    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        geometry_msgs::msg::TransformStamped transform;
        try {
            transform = tf_buffer_->lookupTransform(target_frame_, source_frame_, tf2::TimePointZero);
        } catch (const tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s", ex.what());
            return;
        }

        float theta = std::asin(std::clamp(2.0f * (transform.transform.rotation.w * transform.transform.rotation.y -
                                                  transform.transform.rotation.z * transform.transform.rotation.x), -1.0f, 1.0f));

        size_t n = msg->ranges.size();
        std::vector<float> filtered_ranges = msg->ranges;

        float cos_theta = std::cos(theta);
        float sin_theta = std::sin(theta);
        float tx = transform.transform.translation.x;
        float ty = transform.transform.translation.y;

        for (size_t i = 0; i < n; ++i) {
            float r = msg->ranges[i];
            if (!std::isfinite(r)) {
                filtered_ranges[i] = default_bad_range_;
                continue;
            }
            float angle = msg->angle_min + i * msg->angle_increment;
            float x = r * std::cos(angle);
            float y = r * std::sin(angle);

            float x_rot = cos_theta * x - sin_theta * y + tx;
            float y_rot = sin_theta * x + cos_theta * y + ty;

            if (x_rot < min_x_ || x_rot > max_x_ || y_rot < min_y_ || y_rot > max_y_) {
                filtered_ranges[i] = default_bad_range_;
            }
        }

        auto out_scan = *msg;
        out_scan.ranges = filtered_ranges;
        scan_pub_->publish(out_scan);
    }

    std::string target_frame_;
    std::string source_frame_;
    float min_x_, min_y_, max_x_, max_y_;
    float default_bad_range_;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr kurome_scan_sub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;

    rclcpp::Subscription<kurome::msg::Rectangle>::SharedPtr arena_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WallFilterNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

