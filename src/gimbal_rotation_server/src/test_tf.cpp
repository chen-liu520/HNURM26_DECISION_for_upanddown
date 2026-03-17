#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <Eigen/Dense>

namespace gimbal_rotation_server
{

    class TestNode : public rclcpp::Node
    {
        public:
            TestNode() : Node("test_tf_node")
            {
                timer_ = this->create_wall_timer(
                    std::chrono::milliseconds(1200),
                    std::bind(&TestNode::timer_callback, this));
                tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
                tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
            }
            ~TestNode(){

            }
            std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
            std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
            rclcpp::TimerBase::SharedPtr timer_;

        private:
            void timer_callback()
            {
                auto start_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
                geometry_msgs::msg::TransformStamped map_to_base_footprint;
                try
                {
                    // 使用50ms前的TF数据，平衡实时性和查询速度
                    rclcpp::Time target_time = now() - rclcpp::Duration::from_seconds(0.05);
                    map_to_base_footprint = tf_buffer_->lookupTransform(
                        "map",
                        "base_footprint",
                        target_time,
                        rclcpp::Duration::from_seconds(0.01));
                }
                catch (const tf2::TransformException &e)
                {
                    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 500,
                                         "\033[36m[NavigateInUpAndDownArea] TF查询失败: %s\033[0m", e.what());
                }
                auto end_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
                auto duration = end_time - start_time;
                RCLCPP_INFO(get_logger(), "TF查询耗时: %ld 毫秒", duration);
            }
    };
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<gimbal_rotation_server::TestNode>();

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}