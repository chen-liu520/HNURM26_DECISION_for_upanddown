#include <rclcpp/rclcpp.hpp>
#include <hnurm_interfaces/msg/gesture.hpp>
#include <string>

namespace gimbal_rotation_server
{

    class GesturePrinterNode : public rclcpp::Node
    {
    public:
        GesturePrinterNode()
            : Node("gesture_printer_node"),
              last_gesture_(255), // 初始化为无效值
              print_count_(0)
        {
            RCLCPP_INFO(this->get_logger(), "\033[32m[GesturePrinter] 姿态打印节点启动\033[0m");

            // 声明参数
            this->declare_parameter("print_interval", 1);     // 打印间隔（每隔N条消息打印一次）
            this->declare_parameter("print_on_change", true); // 仅在姿态变化时打印
            this->declare_parameter("show_counter", true);    // 显示消息计数器

            // 获取参数
            print_interval_ = this->get_parameter("print_interval").as_int();
            print_on_change_ = this->get_parameter("print_on_change").as_bool();
            show_counter_ = this->get_parameter("show_counter").as_bool();

            // 创建订阅者 - 订阅姿态消息
            gesture_sub_ = this->create_subscription<hnurm_interfaces::msg::Gesture>(
                "/decision/gesture",
                rclcpp::SensorDataQoS(),
                std::bind(&GesturePrinterNode::gesture_callback, this, std::placeholders::_1));

            RCLCPP_INFO(this->get_logger(), "\033[32m[GesturePrinter] 初始化完成\033[0m");
            RCLCPP_INFO(this->get_logger(), "  订阅话题: /decision/gesture");
            RCLCPP_INFO(this->get_logger(), "  打印间隔: 每 %d 条消息", print_interval_);
            RCLCPP_INFO(this->get_logger(), "  变化打印: %s", print_on_change_ ? "是" : "否");
        }

    private:
        /**
         * @brief Gesture 消息回调函数
         * @param msg Gesture 消息
         */
        void gesture_callback(const hnurm_interfaces::msg::Gesture::SharedPtr msg)
        {
            print_count_++;

            // 获取姿态字符串
            std::string gesture_str;
            std::string gesture_color;
            switch (msg->pose)
            {
            case hnurm_interfaces::msg::Gesture::ATTACK:
                gesture_str = "ATTACK(攻击)";
                gesture_color = "\033[1;31m"; // 红色
                break;
            case hnurm_interfaces::msg::Gesture::MOVE:
                gesture_str = "MOVE(移动)";
                gesture_color = "\033[1;34m"; // 蓝色
                break;
            case hnurm_interfaces::msg::Gesture::DEFEND:
                gesture_str = "DEFEND(防御)";
                gesture_color = "\033[1;32m"; // 绿色
                break;
            default:
                gesture_str = "UNKNOWN(未知:" + std::to_string(msg->pose) + ")";
                gesture_color = "\033[1;33m"; // 黄色
                break;
            }

            // 判断是否需要打印
            bool should_print = false;

            if (print_on_change_)
            {
                // 仅在姿态变化时打印
                if (msg->pose != last_gesture_)
                {
                    should_print = true;
                }
            }
            else
            {
                // 按间隔打印
                if (print_count_ % print_interval_ == 0)
                {
                    should_print = true;
                }
            }

            // 打印姿态信息
            if (should_print)
            {
                if (show_counter_)
                {
                    RCLCPP_INFO(this->get_logger(),
                                "%s[GESTURE] [#%zu] 当前姿态: %s\033[0m",
                                gesture_color.c_str(), print_count_, gesture_str.c_str());
                }
                else
                {
                    RCLCPP_INFO(this->get_logger(),
                                "%s[GESTURE] 当前姿态: %s\033[0m",
                                gesture_color.c_str(), gesture_str.c_str());
                }

                // 记录上次姿态
                last_gesture_ = msg->pose;
            }
        }

    private:
        // 订阅者
        rclcpp::Subscription<hnurm_interfaces::msg::Gesture>::SharedPtr gesture_sub_;

        // 状态
        uint8_t last_gesture_;
        size_t print_count_;

        // 参数
        int print_interval_;
        bool print_on_change_;
        bool show_counter_;
    };

} // namespace gimbal_rotation_server

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<gimbal_rotation_server::GesturePrinterNode>();

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}