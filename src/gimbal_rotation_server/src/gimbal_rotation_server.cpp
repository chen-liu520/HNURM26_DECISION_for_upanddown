/**
 * @file gimbal_rotation_server.cpp
 * @brief 云台旋转180度服务端
 * 
 * 统一接口: /decision/back_target_rotate
 * 
 * 支持两种触发方式（二选一）:
 * 1. 服务触发: /decision/back_target_rotate (std_srvs/srv/Trigger)
 * 2. 话题触发: /decision/back_target_rotate (可配置消息类型)
 *    - std_msgs::Bool   - true触发旋转180度
 *    - std_msgs::Float32 - 目标角度（度）
 *    - geometry_msgs::Vector3 - x=yaw, y=pitch, z=speed
 */

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <cmath>

namespace gimbal_rotation
{

class GimbalRotationServer : public rclcpp::Node
{
public:
    GimbalRotationServer()
    : Node("gimbal_rotation_server"),
      current_yaw_(0.0)
    {
        // 声明参数
        this->declare_parameter("rotation_speed", 90.0);  // 默认旋转速度：90度/秒
        this->declare_parameter("current_yaw", 0.0);      // 当前yaw角
        this->declare_parameter("current_pitch", 0.0);    // 当前pitch角
        this->declare_parameter("topic_type", "none");    // 话题类型: "none"/"bool"/"float32"/"vector3"
        
        // ==================== 服务服务端 ====================
        // 统一接口: /decision/back_target_rotate
        // 类型: std_srvs/srv/Trigger
        service_ = this->create_service<std_srvs::srv::Trigger>(
            "/decision/back_target_rotate",
            std::bind(&GimbalRotationServer::handleServiceRequest, this,
                      std::placeholders::_1, std::placeholders::_2));
        
        RCLCPP_INFO(this->get_logger(), 
                    "\033[1;32m[GimbalRotationServer] Service started: /decision/back_target_rotate\033[0m");
        
        // ==================== 话题订阅（已注释，按需启用）====================
        // 话题名与服务名相同: /decision/back_target_rotate
        // 根据需要取消注释对应的订阅，注意：同一时间建议只使用服务或话题之一
        
        /*
        // 方案1: Bool触发
        // 发布方式: std_msgs::Bool, data=true 触发旋转180度
        bool_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/decision/back_target_rotate",
            10,
            std::bind(&GimbalRotationServer::handleBoolTrigger, this, std::placeholders::_1));
        
        RCLCPP_INFO(this->get_logger(), 
                    "\033[1;33m[GimbalRotationServer] Topic subscriber enabled (Bool): /decision/back_target_rotate\033[0m");
        */
        
        /*
        // 方案2: Float32目标角度
        // 发布方式: std_msgs::Float32, data=目标角度（度）
        angle_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/decision/back_target_rotate",
            10,
            std::bind(&GimbalRotationServer::handleTargetAngle, this, std::placeholders::_1));
        
        RCLCPP_INFO(this->get_logger(), 
                    "\033[1;33m[GimbalRotationServer] Topic subscriber enabled (Float32): /decision/back_target_rotate\033[0m");
        */
        
        /*
        // 方案3: Vector3完整控制
        // 发布方式: geometry_msgs::Vector3, x=yaw, y=pitch, z=speed
        cmd_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
            "/decision/back_target_rotate",
            10,
            std::bind(&GimbalRotationServer::handleCommand, this, std::placeholders::_1));
        
        RCLCPP_INFO(this->get_logger(), 
                    "\033[1;33m[GimbalRotationServer] Topic subscriber enabled (Vector3): /decision/back_target_rotate\033[0m");
        */
        
        RCLCPP_INFO(this->get_logger(), 
                    "\033[1;35m=================================================\033[0m");
        RCLCPP_INFO(this->get_logger(), 
                    "\033[1;35m[GimbalRotationServer] Node initialization complete\033[0m");
        RCLCPP_INFO(this->get_logger(), 
                    "\033[1;35mInterface: /decision/back_target_rotate\033[0m");
        RCLCPP_INFO(this->get_logger(), 
                    "\033[1;35m=================================================\033[0m");
    }

private:
    // ==================== 服务回调 ====================
    void handleServiceRequest(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        (void)request;  // 请求参数为空，无需处理
        
        RCLCPP_INFO(this->get_logger(), 
                    "\033[1;35m[Service] Received rotate request on /decision/back_target_rotate\033[0m");
        
        // 获取当前参数
        //double rotation_speed = this->get_parameter("rotation_speed").as_double();
        current_yaw_ = this->get_parameter("current_yaw").as_double();
        
        // 计算目标角度（当前角度 + 180度）
        double target_yaw = current_yaw_ + 180.0;
        normalizeAngle(target_yaw);
    
        
        // ==========================================================
        // 【处理留白】实际云台控制代码写在这里
        // TODO: 调用硬件接口或发送CAN/串口命令
        // 
        // 示例伪代码：
        // gimbal_hardware_.rotateTo(target_yaw, current_pitch_, rotation_speed);
        // 或
        // can_bus_.send_gimbal_command(target_yaw, current_pitch_, rotation_speed);
        // ==========================================================
        
        RCLCPP_WARN(this->get_logger(), 
                    "\033[1;35m[Service] >>> HARDWARE CONTROL NOT IMPLEMENTED <<<\033[0m");
        RCLCPP_INFO(this->get_logger(), 
                    "\033[1;36m[Service] Place your gimbal control code in the marked area\033[0m");
        
        // 模拟更新当前角度（实际应从硬件反馈获取）
        current_yaw_ = target_yaw;
        this->set_parameter(rclcpp::Parameter("current_yaw", current_yaw_));
        
        // 设置响应
        response->success = true;
        response->message = std::string("Rotation command accepted: ") +
                           std::to_string(static_cast<int>(current_yaw_)) + " -> " + 
                           std::to_string(static_cast<int>(target_yaw));
        
        RCLCPP_INFO(this->get_logger(), 
                    "\033[1;32m[Service] Request processed successfully\033[0m");
    }
    
    // ==================== 话题回调 - Bool触发（已注释）====================
    void handleBoolTrigger(const std_msgs::msg::Bool::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), 
                    "\033[1;33m[Topic:Bool] Received on /decision/back_target_rotate: %s\033[0m",
                    msg->data ? "true" : "false");
        
        if (!msg->data) {
            RCLCPP_INFO(this->get_logger(), "[Topic:Bool] Trigger is false, ignoring");
            return;
        }
        
        //double rotation_speed = this->get_parameter("rotation_speed").as_double();
        current_yaw_ = this->get_parameter("current_yaw").as_double();
        
        double target_yaw = current_yaw_ + 180.0;
        normalizeAngle(target_yaw);
        
        RCLCPP_INFO(this->get_logger(), 
                    "\033[1;34m[Topic:Bool] Rotation: %.2f° -> %.2f°\033[0m",
                    current_yaw_, target_yaw);
        
        // ==========================================================
        // 【处理留白】实际云台控制代码写在这里
        // ==========================================================
        
        RCLCPP_WARN(this->get_logger(), 
                    "\033[1;35m[Topic:Bool] >>> HARDWARE CONTROL NOT IMPLEMENTED <<<\033[0m");
        
        current_yaw_ = target_yaw;
        this->set_parameter(rclcpp::Parameter("current_yaw", current_yaw_));
        
        RCLCPP_INFO(this->get_logger(), "\033[1;32m[Topic:Bool] Processed\033[0m");
    }
    
    // ==================== 话题回调 - Float32目标角度（已注释）====================
    void handleTargetAngle(const std_msgs::msg::Float32::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), 
                    "\033[1;33m[Topic:Angle] Received on /decision/back_target_rotate: %.2f°\033[0m",
                    msg->data);
        
        //double rotation_speed = this->get_parameter("rotation_speed").as_double();
        current_yaw_ = this->get_parameter("current_yaw").as_double();
        
        double target_yaw = msg->data;
        normalizeAngle(target_yaw);
        
        RCLCPP_INFO(this->get_logger(), 
                    "\033[1;34m[Topic:Angle] Rotation: %.2f° -> %.2f°\033[0m",
                    current_yaw_, target_yaw);
        
        // ==========================================================
        // 【处理留白】实际云台控制代码写在这里
        // ==========================================================
        
        RCLCPP_WARN(this->get_logger(), 
                    "\033[1;35m[Topic:Angle] >>> HARDWARE CONTROL NOT IMPLEMENTED <<<\033[0m");
        
        current_yaw_ = target_yaw;
        this->set_parameter(rclcpp::Parameter("current_yaw", current_yaw_));
        
        RCLCPP_INFO(this->get_logger(), "\033[1;32m[Topic:Angle] Processed\033[0m");
    }
    
    // ==================== 话题回调 - Vector3完整控制（已注释）====================
    void handleCommand(const geometry_msgs::msg::Vector3::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), 
                    "\033[1;33m[Topic:Command] Received on /decision/back_target_rotate: "
                    "yaw=%.2f°, pitch=%.2f°, speed=%.2f°/s\033[0m",
                    msg->x, msg->y, msg->z);
        
        current_yaw_ = this->get_parameter("current_yaw").as_double();
        
        double target_yaw = msg->x;
        double target_pitch = msg->y;
        //double rotation_speed = msg->z;
        
        RCLCPP_INFO(this->get_logger(), 
                    "\033[1;34m[Topic:Command] Current: %.2f° -> Target: %.2f°\033[0m",
                    current_yaw_, target_yaw);
        
        // ==========================================================
        // 【处理留白】实际云台控制代码写在这里
        // ==========================================================
        
        RCLCPP_WARN(this->get_logger(), 
                    "\033[1;35m[Topic:Command] >>> HARDWARE CONTROL NOT IMPLEMENTED <<<\033[0m");
        
        current_yaw_ = target_yaw;
        normalizeAngle(current_yaw_);
        this->set_parameter(rclcpp::Parameter("current_yaw", current_yaw_));
        this->set_parameter(rclcpp::Parameter("current_pitch", target_pitch));
        
        RCLCPP_INFO(this->get_logger(), "\033[1;32m[Topic:Command] Processed\033[0m");
    }
    
    // 工具函数：将角度归一化到 [0, 360)
    void normalizeAngle(double& angle)
    {
        while (angle >= 360.0) angle -= 360.0;
        while (angle < 0.0) angle += 360.0;
    }

private:
    // 服务端
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_;
    
    // 话题订阅（按需启用）
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr bool_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr angle_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr cmd_sub_;
    
    // 状态
    double current_yaw_;
};

} // namespace gimbal_rotation

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<gimbal_rotation::GimbalRotationServer>();
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}
