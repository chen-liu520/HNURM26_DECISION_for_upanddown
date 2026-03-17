/**
 * @file referee.cpp
 * @brief 模拟裁判系统ROS2节点
 * 
 * 功能：
 * 1. 发布 VisionRecvData 消息，支持手动输入血量值
 * 2. 订阅 Gesture 消息，回调打印当前姿态，并更新到 VisionRecvData 发布
 */

#include <rclcpp/rclcpp.hpp>
#include <hnurm_interfaces/msg/vision_recv_data.hpp>
#include <hnurm_interfaces/msg/gesture.hpp>
#include <std_msgs/msg/header.hpp>
#include <thread>
#include <atomic>
#include <iostream>

namespace gimbal_rotation_server
{

class RefereeNode : public rclcpp::Node
{
public:
    RefereeNode() : Node("referee_node"), current_gesture_(hnurm_interfaces::msg::Gesture::MOVE)
    {
        RCLCPP_INFO(this->get_logger(), "\033[32m[RefereeNode] 模拟裁判系统节点启动\033[0m");

        // 声明参数（血量值）
        this->declare_parameter("current_hp", 400.0);
        this->declare_parameter("current_base_hp", 5000.0);
        this->declare_parameter("current_enemy_base_hp", 5000.0);
        this->declare_parameter("current_outpost_hp", 1500.0);
        this->declare_parameter("current_enemy_outpost_hp", 1500.0);
        this->declare_parameter("allow_fire_amount", 500.0);
        this->declare_parameter("game_progress", 0.1);  // 比赛阶段
        this->declare_parameter("self_color", 0);       // 1=RED, 2=BLUE
        this->declare_parameter("publish_rate", 1.0);  // 发布频率(Hz)

        // 创建发布者
        vision_recv_pub_ = this->create_publisher<hnurm_interfaces::msg::VisionRecvData>(
            "/hp_recv_data", 10);

        // 创建订阅者 - 订阅姿态消息
        gesture_sub_ = this->create_subscription<hnurm_interfaces::msg::Gesture>(
            "/decision/gesture",
            rclcpp::SensorDataQoS(),
            std::bind(&RefereeNode::gesture_callback, this, std::placeholders::_1));

        // 创建定时器发布 VisionRecvData
        double publish_rate = this->get_parameter("publish_rate").as_double();
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate)),
            std::bind(&RefereeNode::publish_vision_recv_data, this));

        // 启动输入线程用于手动修改血量
        input_thread_ = std::thread(&RefereeNode::input_thread_func, this);

        RCLCPP_INFO(this->get_logger(), "\033[32m[RefereeNode] 初始化完成\033[0m");
        RCLCPP_INFO(this->get_logger(), "命令帮助：");
        RCLCPP_INFO(this->get_logger(), "  hp <值>      - 设置当前血量");
        RCLCPP_INFO(this->get_logger(), "  base <值>    - 设置我方基地血量");
        RCLCPP_INFO(this->get_logger(), "  ebase <值>   - 设置敌方基地血量");
        RCLCPP_INFO(this->get_logger(), "  outpost <值> - 设置我方前哨站血量");
        RCLCPP_INFO(this->get_logger(), "  eoutpost <值>- 设置敌方前哨站血量");
        RCLCPP_INFO(this->get_logger(), "  fire <值>    - 设置允许发射弹量");
        RCLCPP_INFO(this->get_logger(), "  progress <值>- 设置游戏进度(>0.5开始游戏)");
        RCLCPP_INFO(this->get_logger(), "  show         - 显示当前所有状态");
        RCLCPP_INFO(this->get_logger(), "  help         - 显示帮助信息");
    }

    ~RefereeNode()
    {
        running_ = false;
        if (input_thread_.joinable()) {
            input_thread_.join();
        }
    }

private:
    // 发布者
    rclcpp::Publisher<hnurm_interfaces::msg::VisionRecvData>::SharedPtr vision_recv_pub_;
    
    // 订阅者
    rclcpp::Subscription<hnurm_interfaces::msg::Gesture>::SharedPtr gesture_sub_;
    
    // 定时器
    rclcpp::TimerBase::SharedPtr timer_;

    // 当前姿态
    std::atomic<uint8_t> current_gesture_;
    std::mutex gesture_mutex_;

    // 输入线程控制
    std::atomic<bool> running_{true};
    std::thread input_thread_;

    /**
     * @brief Gesture 消息回调函数
     * @param msg Gesture 消息
     */
    void gesture_callback(const hnurm_interfaces::msg::Gesture::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(gesture_mutex_);
        current_gesture_ = msg->pose;
        
        // 打印当前姿态
        std::string gesture_str;
        switch (msg->pose) {
            case hnurm_interfaces::msg::Gesture::ATTACK:
                gesture_str = "ATTACK(攻击)";
                break;
            case hnurm_interfaces::msg::Gesture::MOVE:
                gesture_str = "MOVE(移动)";
                break;
            case hnurm_interfaces::msg::Gesture::DEFEND:
                gesture_str = "DEFEND(防御)";
                break;
            default:
                gesture_str = "UNKNOWN(未知:" + std::to_string(msg->pose) + ")";
                break;
        }
        
        RCLCPP_INFO(this->get_logger(), "\033[36m[Gesture回调] 收到姿态: %s\033[0m", gesture_str.c_str());
    }

    /**
     * @brief 发布 VisionRecvData 消息
     */
    void publish_vision_recv_data()
    {
        auto msg = hnurm_interfaces::msg::VisionRecvData();
        
        // 获取当前 game_progress
        double current_progress = this->get_parameter("game_progress").as_double();
        
        // 调试：如果 game_progress 变化，打印日志
        static double last_progress = -1.0;
        if (std::abs(current_progress - last_progress) > 0.01) {
            RCLCPP_INFO(this->get_logger(), "[发布数据] game_progress 变化: %.1f -> %.1f", 
                       last_progress, current_progress);
            last_progress = current_progress;
        }
        
        // 填充消息头
        msg.header.stamp = this->now();
        msg.header.frame_id = "referee";

        // 从参数获取血量值
        msg.current_hp = this->get_parameter("current_hp").as_double();
        msg.current_base_hp = this->get_parameter("current_base_hp").as_double();
        msg.current_enemy_base_hp = this->get_parameter("current_enemy_base_hp").as_double();
        msg.current_outpost_hp = this->get_parameter("current_outpost_hp").as_double();
        msg.current_enemy_outpost_hp = this->get_parameter("current_enemy_outpost_hp").as_double();
        msg.allow_fire_amount = this->get_parameter("allow_fire_amount").as_double();
        msg.game_progress = this->get_parameter("game_progress").as_double();

        // 设置颜色
        msg.self_color.data = this->get_parameter("self_color").as_int();

        // 设置姿态（从订阅的Gesture消息获取）
        {
            std::lock_guard<std::mutex> lock(gesture_mutex_);
            msg.gesture.pose = current_gesture_.load();
        }

        // 设置其他默认值
        msg.roll = 0.0;
        msg.pitch = 0.0;
        msg.yaw = 0.0;
        msg.control_id = 0.0;
        msg.hero_x = 0.0;
        msg.hero_y = 0.0;
        msg.cmd_x = 0.0;
        msg.cmd_y = 0.0;

        // 发布消息
        vision_recv_pub_->publish(msg);
    }

    /**
     * @brief 输入线程函数，用于手动输入血量值
     */
    void input_thread_func()
    {
        std::string input;
        
        while (running_ && rclcpp::ok()) {
            std::cout << "\033[33m[Referee] 输入命令 (help查看帮助): \033[0m";
            std::cout.flush();
            
            if (!std::getline(std::cin, input)) {
                break;
            }

            // 解析命令
            std::istringstream iss(input);
            std::string cmd;
            iss >> cmd;

            if (cmd == "hp") {
                double value;
                if (iss >> value) {
                    this->set_parameter(rclcpp::Parameter("current_hp", value));
                    RCLCPP_INFO(this->get_logger(), "\033[32m[参数更新] 当前血量设置为: %.1f\033[0m", value);
                } else {
                    RCLCPP_WARN(this->get_logger(), "用法: hp <值>");
                }
            }
            else if (cmd == "base") {
                double value;
                if (iss >> value) {
                    this->set_parameter(rclcpp::Parameter("current_base_hp", value));
                    RCLCPP_INFO(this->get_logger(), "\033[32m[参数更新] 我方基地血量设置为: %.1f\033[0m", value);
                } else {
                    RCLCPP_WARN(this->get_logger(), "用法: base <值>");
                }
            }
            else if (cmd == "ebase") {
                double value;
                if (iss >> value) {
                    this->set_parameter(rclcpp::Parameter("current_enemy_base_hp", value));
                    RCLCPP_INFO(this->get_logger(), "\033[32m[参数更新] 敌方基地血量设置为: %.1f\033[0m", value);
                } else {
                    RCLCPP_WARN(this->get_logger(), "用法: ebase <值>");
                }
            }
            else if (cmd == "outpost") {
                double value;
                if (iss >> value) {
                    this->set_parameter(rclcpp::Parameter("current_outpost_hp", value));
                    RCLCPP_INFO(this->get_logger(), "\033[32m[参数更新] 我方前哨站血量设置为: %.1f\033[0m", value);
                } else {
                    RCLCPP_WARN(this->get_logger(), "用法: outpost <值>");
                }
            }
            else if (cmd == "eoutpost") {
                double value;
                if (iss >> value) {
                    this->set_parameter(rclcpp::Parameter("current_enemy_outpost_hp", value));
                    RCLCPP_INFO(this->get_logger(), "\033[32m[参数更新] 敌方前哨站血量设置为: %.1f\033[0m", value);
                } else {
                    RCLCPP_WARN(this->get_logger(), "用法: eoutpost <值>");
                }
            }
            else if (cmd == "fire") {
                double value;
                if (iss >> value) {
                    this->set_parameter(rclcpp::Parameter("allow_fire_amount", value));
                    RCLCPP_INFO(this->get_logger(), "\033[32m[参数更新] 允许发射弹量设置为: %.1f\033[0m", value);
                } else {
                    RCLCPP_WARN(this->get_logger(), "用法: fire <值>");
                }
            }
            else if (cmd == "progress") {
                double value;
                if (iss >> value) {
                    this->set_parameter(rclcpp::Parameter("game_progress", value));
                    RCLCPP_INFO(this->get_logger(), "\033[32m[参数更新] 游戏进度设置为: %.1f (%s)\033[0m", 
                               value, value > 0.5 ? "游戏中" : "未开始");
                } else {
                    RCLCPP_WARN(this->get_logger(), "用法: progress <值> (例: 4.0)");
                }
            }
            else if (cmd == "color") {
                int value;
                if (iss >> value) {
                    this->set_parameter(rclcpp::Parameter("self_color", value));
                    RCLCPP_INFO(this->get_logger(), "\033[32m[参数更新] 我方颜色设置为: %d (%s)\033[0m", 
                               value, value == 1 ? "RED" : (value == 2 ? "BLUE" : "UNKNOWN"));
                } else {
                    RCLCPP_WARN(this->get_logger(), "用法: color <1=RED, 2=BLUE>");
                }
            }
            else if (cmd == "show") {
                show_current_status();
            }
            else if (cmd == "help" || cmd == "h") {
                print_help();
            }
            else if (cmd.empty()) {
                // 空输入，忽略
            }
            else {
                RCLCPP_WARN(this->get_logger(), "未知命令: %s, 输入 help 查看帮助", cmd.c_str());
            }
        }
    }

    /**
     * @brief 显示当前状态
     */
    void show_current_status()
    {
        RCLCPP_INFO(this->get_logger(), "\033[35m========== 当前状态 ==========");
        RCLCPP_INFO(this->get_logger(), "当前血量: %.1f", this->get_parameter("current_hp").as_double());
        RCLCPP_INFO(this->get_logger(), "我方基地血量: %.1f", this->get_parameter("current_base_hp").as_double());
        RCLCPP_INFO(this->get_logger(), "敌方基地血量: %.1f", this->get_parameter("current_enemy_base_hp").as_double());
        RCLCPP_INFO(this->get_logger(), "我方前哨站血量: %.1f", this->get_parameter("current_outpost_hp").as_double());
        RCLCPP_INFO(this->get_logger(), "敌方前哨站血量: %.1f", this->get_parameter("current_enemy_outpost_hp").as_double());
        RCLCPP_INFO(this->get_logger(), "允许发射弹量: %.1f", this->get_parameter("allow_fire_amount").as_double());
        RCLCPP_INFO(this->get_logger(), "游戏进度: %.1f (%s)", this->get_parameter("game_progress").as_double(),
                   this->get_parameter("game_progress").as_double() > 0.5 ? "游戏中" : "未开始");
        
        int color = this->get_parameter("self_color").as_int();
        RCLCPP_INFO(this->get_logger(), "我方颜色: %d (%s)", color, color == 1 ? "RED" : (color == 2 ? "BLUE" : "UNKNOWN"));
        
        std::string gesture_str;
        switch (current_gesture_.load()) {
            case hnurm_interfaces::msg::Gesture::ATTACK: gesture_str = "ATTACK(攻击)"; break;
            case hnurm_interfaces::msg::Gesture::MOVE: gesture_str = "MOVE(移动)"; break;
            case hnurm_interfaces::msg::Gesture::DEFEND: gesture_str = "DEFEND(防御)"; break;
            default: gesture_str = "UNKNOWN(" + std::to_string(current_gesture_.load()) + ")"; break;
        }
        RCLCPP_INFO(this->get_logger(), "当前姿态: %s\033[0m", gesture_str.c_str());
    }

    /**
     * @brief 打印帮助信息
     */
    void print_help()
    {
        RCLCPP_INFO(this->get_logger(), "\033[35m========== 命令帮助 ==========");
        RCLCPP_INFO(this->get_logger(), "hp <值>       - 设置当前血量");
        RCLCPP_INFO(this->get_logger(), "base <值>     - 设置我方基地血量");
        RCLCPP_INFO(this->get_logger(), "ebase <值>    - 设置敌方基地血量");
        RCLCPP_INFO(this->get_logger(), "outpost <值>  - 设置我方前哨站血量");
        RCLCPP_INFO(this->get_logger(), "eoutpost <值> - 设置敌方前哨站血量");
        RCLCPP_INFO(this->get_logger(), "fire <值>     - 设置允许发射弹量");
        RCLCPP_INFO(this->get_logger(), "progress <值> - 设置游戏进度 (>0.5开始游戏)");
        RCLCPP_INFO(this->get_logger(), "color <1/2>   - 设置颜色 (1=RED, 2=BLUE)");
        RCLCPP_INFO(this->get_logger(), "show          - 显示当前所有状态");
        RCLCPP_INFO(this->get_logger(), "help          - 显示此帮助信息");
        RCLCPP_INFO(this->get_logger(), "==============================\033[0m");
    }
};

}  // namespace gimbal_rotation_server

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<gimbal_rotation_server::RefereeNode>();
    
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}
