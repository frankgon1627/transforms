#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

class StaticTransformPublisher : public rclcpp::Node {
public:
    StaticTransformPublisher() : Node("tf_static_broadcaster")
    {
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/dlio/odom_node/odom", 10, std::bind(&StaticTransformPublisher::odom_callback, this, std::placeholders::_1));

        publish_lidar_transform();
    }

private:
    void(odom_callback)(const nav_msgs::msg::Odometry::SharedPtr msg){
        odom_ = msg;
        publish_map_transform();
    }

    void publish_lidar_transform() {
        geometry_msgs::msg::TransformStamped lidar_transform;

        lidar_transform.header.stamp = this->get_clock()->now();
        lidar_transform.header.frame_id = "base_link";  // Parent frame
        lidar_transform.child_frame_id = "os_sensor";   // Child frame

        // Set translation (x, y, z)
        lidar_transform.transform.translation.x = 0.15;
        lidar_transform.transform.translation.y = 0.0;
        lidar_transform.transform.translation.z = 0.158;

        // Set rotation (quaternion)
        lidar_transform.transform.rotation.x = 0.0;
        lidar_transform.transform.rotation.y = 0.0;
        lidar_transform.transform.rotation.z = 0.0;
        lidar_transform.transform.rotation.w = 1.0;

        // Broadcast the transform
        static_tf_broadcaster_->sendTransform(lidar_transform);

        RCLCPP_INFO(this->get_logger(), "Published static transform from 'base_link' to 'os_sensor'");
    }

    void publish_map_transform(){
        geometry_msgs::msg::TransformStamped map_transform;

        map_transform.header.stamp = odom_->header.stamp;
        map_transform.header.frame_id = "map";  // Parent frame
        map_transform.child_frame_id = "base_link";   // Child frame

        // Set translation (x, y, z)
        map_transform.transform.translation.x = odom_->pose.pose.position.x;
        map_transform.transform.translation.y = odom_->pose.pose.position.y;
        map_transform.transform.translation.z = odom_->pose.pose.position.z;

        // Set rotation (quaternion)
        map_transform.transform.rotation.x = odom_->pose.pose.orientation.x;
        map_transform.transform.rotation.y = odom_->pose.pose.orientation.y;
        map_transform.transform.rotation.z = odom_->pose.pose.orientation.z;
        map_transform.transform.rotation.w = odom_->pose.pose.orientation.w;

        // Broadcast the transform
        tf_broadcaster_->sendTransform(map_transform);

        RCLCPP_INFO(this->get_logger(), "Published static transform from 'map' to 'base_link'");
    }

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    nav_msgs::msg::Odometry::SharedPtr odom_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<StaticTransformPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
