#pragma once

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "hnurm_interfaces/msg/vision_recv_data.hpp"
#include "hnurm_interfaces/msg/vision_send_data.hpp"
#include "hnurm_interfaces/msg/target.hpp"
#include <hnurm_interfaces/msg/special_area.hpp>
#include <hnurm_interfaces/msg/area.hpp>
#include <hnurm_interfaces/msg/type.hpp>
#include <hnurm_interfaces/msg/self_color.hpp>
#include "std_msgs/msg/float32.hpp"
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include "std_srvs/srv/trigger.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <Eigen/Dense>

#include <deque>
#include <mutex>
#include <atomic>
#include <vector>
#include <algorithm>
#include <unordered_map>
#include <cmath>
#include <stdexcept>

namespace hnurm_behavior_trees
{

    class PubRobotStatus : public BT::SyncActionNode
    {
    public:
        PubRobotStatus(const std::string &xml_tag_name, const BT::NodeConfiguration &conf);
        ~PubRobotStatus() override;

        static BT::PortsList providedPorts()
        {
            // BT::OutputPort<bool>("黑板变量名", "黑板变量描述"),
            // setOutput("黑板变量名"， 变量值)
            return {
                // 状态机
                BT::OutputPort<bool>("pose_from_human", "pose_from_human_interrupt"), // 标记是否接受到云台手发送的目标点
                BT::OutputPort<bool>("need_supply", "need_supply"),                   // 标记是否需要补给
                BT::OutputPort<bool>("is_pursue", "is_pursue"),                       // 标记是否需要追击
                BT::OutputPort<bool>("is_cruise", "is_cruise"),                       // 标记是否需要巡航
                BT::OutputPort<bool>("is_in_upanddown_area", "is_in_upanddown_area"), // 标记是否在起伏路段区域内

                // 目标点
                BT::OutputPort<geometry_msgs::msg::PoseStamped>("upanddown_goal", "upanddown area goal pose"),
                BT::OutputPort<geometry_msgs::msg::PoseStamped>("human_goal", "human goal pose"),
                BT::OutputPort<geometry_msgs::msg::PoseStamped>("supply_goal", "supply goal pose"),
                BT::OutputPort<geometry_msgs::msg::PoseStamped>("pursue_goal", "pursue goal pose"),
                BT::OutputPort<geometry_msgs::msg::PoseStamped>("cruise_goal", "cruise goal pose"),

                // 其他目标点队列

                // 其他
                BT::OutputPort<std::string>("current_controller", "current_controller"),
            };
        }

        BT::NodeStatus tick() override;

        rclcpp::Node::SharedPtr node_;

        rclcpp::CallbackGroup::SharedPtr callback_group_; // 常规回调组
        rclcpp::executors::SingleThreadedExecutor callback_group_executor_;

        rclcpp::SubscriptionOptions sub_option;

        // 全局坐标点
        struct GlobalPose
        {
            float pose_x;
            float pose_y;
        };

        hnurm_interfaces::msg::VisionSendData send_target_information_;
        hnurm_interfaces::msg::VisionRecvData recv_robot_information_;

        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

        enum class SelfColor
        {
            RED,
            BLUE,
        };
        SelfColor self_color_; // 我方颜色

        bool is_color_setted_ = false; // 是否已经从串口状态消息中获取到颜色信息

        bool is_init_ = false;

        bool is_stop_ = false; // 标记是否需要停止

    private:
        // 前向声明枚举类型（在函数声明之前定义）
        enum class CruiseMode
        {
            HUMAN,         // 云台手发送的目标点
            SUPPLY,        // 补给点
            MY_FORT,       // 我方堡垒点
            OPPONENT_FORT, // 敌方堡垒点
            MY_HALF,       // 我方半场路径
            OPPONENT_HALF, // 敌方半场路径
            HIGHLAND,        // 高地-默认环绕路径
            HIGHLAND_AMBUSH, // 高地-阻击路径（前哨区域）
            HIGHLAND_MIXED   // 高地-混合路径
        };

        enum class SpecialAreaType
        {
            MOVE_AREA,
            UPANDDOWN_AREA,
            ILLEGAL_AREA
        };

        enum class UpAndDownNavStatus
        {
            NOT_START, 
            SPIN,       // 旋转调整角度
            RELOCATING, // 重定位停车
            LINEAR      // 调整角度后线性行驶
        };

        /*****************************************/
        /*************1. 私有函数 start************/
        /*****************************************/

        // 1.1 订阅回调函数

        void global_pose_callback(const geometry_msgs::msg::PolygonStamped::SharedPtr msg);

        // 重映射速度命令，实现特殊区域内的速度控制
        void remap_cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);

        //  接收视觉系统发送的机器人静态数据和状态信息
        void recv_callback(const hnurm_interfaces::msg::VisionRecvData::SharedPtr msg);

        // 处理后视相机的目标数据（当前代码已注释）
        void back_target_callback(const std_msgs::msg::Float32::SharedPtr msg);

        // 接收视觉系统发送的目标状态信息，控制机器人行为模式切换
        void send_target_info_callback(const hnurm_interfaces::msg::VisionSendData::SharedPtr msg);

        // 巡航路径更新定时器回调函数
        void cruise_change_timer_callback();

        // 特殊区域内相关判断定时器回调函数
        void special_area_timer_callback();

        // 接收云台手发送的目标点，控制机器人切换到human模式
        void human_goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

        // 重定位状态回调函数
        void relocalization_status_callback(const std_msgs::msg::String::SharedPtr msg);

        // 1.2 工具函数
        // 1.2.1 工具最上层
        void CheckIsReached_Update(float threshold); // 检查巡航是否到达目标点，如果到达则更新目标点为队列中的下一个点

        void FillGoalsDeque_Cruise(); // 根据当前状态填充巡航点队列

        void UpdateCurrentCruiseMode(); // 根据当前状态和时间更新巡航模式

        void FillGoalsDeque_Pursue(); // 根据视觉系统发送的目标信息填充追击目标点队列

        void FillGoalsVector_UpAndDown(); // 根据视觉系统发送的目标信息填充起伏路段目标点队列

        SpecialAreaType GetPointArea(const GlobalPose &point); // 获取当前所在区域

        geometry_msgs::msg::Twist NavigateInUpAndDownArea(); // 导航到起伏路段区域Twist控制函数

        // 1.2.2 工具函数细节实现，被最上层和回调调用
        geometry_msgs::msg::PoseStamped GlobalPose2PoseStamped(bool is_pursue);

        GlobalPose calculateAverage(geometry_msgs::msg::PolygonStamped &msg);

        float calculate_distance(const GlobalPose &pose1, const GlobalPose &pose2);

        void structure_get_params(); // 从参数服务器获取参数,填充相关变量，【被PubRobotStatus构造函数调用】

        void structure_get_cruise_paths_from_paramsandcolor(std::string color); // 从参数服务器获取巡航路径点,填充path_pose_arrays_，【被PubRobotStatus构造函数调用】

        void structure_ROS_init(); // 初始化ROS节点、发布者、订阅者，【被PubRobotStatus构造函数调用】

        void initialization(); // 其他初始化工作，【被PubRobotStatus构造函数调用】

        geometry_msgs::msg::Pose get_targetpose_in_map(float &distance);

        void create_pursue_stay_circle_path(); // 创建追击静止小陀螺的路径点队列，围绕目标点做一个半径为0.5m的圆

        bool is_in_this_polygon(const GlobalPose &point, const std::vector<GlobalPose> &polygon);

        CruiseMode stringToCruiseMode(const std::string &mode_str);

        std::string cruiseModeToString(CruiseMode mode);

        std::string upanddownNavStatusToString(const UpAndDownNavStatus &status);

        /*******************************************/
        /***************1. 私有函数 end **************/
        /*******************************************/

        /**************************2. 订阅、发布相关变量 start******************************/
        // 订阅全局【定位】系统发布的机器人位置
        rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr global_pose_sub_;

        // 订阅速度命令,重映射
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

        // 订阅串口发上来的状态信息
        rclcpp::Subscription<hnurm_interfaces::msg::VisionRecvData>::SharedPtr recv_sub_;

        // 订阅视觉发布的目标信息，用于切换追击和巡航状态，以及获取目标距离，目标类型
        rclcpp::Subscription<hnurm_interfaces::msg::VisionSendData>::SharedPtr send_target_info_sub_;

        // 发布速度命令,重映射
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_remap_pub_;

        // 到达起伏路段转向点时发布消息给下位机
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr change_yaw_angle_pub_;

        // 进入起伏路段发布cotrol_id
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr enter_upanddown_ControlID_pub_;

        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr is_in_special_area_pub_; // 发布是否在特殊区域内的消息

        // ROS2定时器
        rclcpp::TimerBase::SharedPtr cruise_change_timer_; // 定时检查是否需要切换巡航路径的定时器

        rclcpp::TimerBase::SharedPtr special_area_timer_; // 特殊区域内相关判断定时器
        // topic名称参数

        std::string cmd_vel_topic_;
        std::string global_position_topic_;
        std::string cmd_vel_remap_topic_;
        std::string gesture_pub_topic_;
        std::string recv_topic_;
        std::string send_topic_;
        std::string back_target_topic_;
        std::string human_goal_topic_;
        std::string yaw_change_topic_;                    // 发布云台旋转命令的话题
        std::string relocalization_topic_;                // 发布重定位服务的话题
        std::string enter_upanddown_ControlID_pub_topic_; // 发布进入起伏路段消息的话题 
        std::string relocalization_status_topic_;                // 订阅重定位状态的话题

        /**************************2. 订阅、发布相关变量 end******************************/

        /*********************3. 巡航相关变量 start ******************************/
        std::unordered_map<CruiseMode, std::vector<GlobalPose>> path_pose_arrays_;

        // 当前的目标点队列
        std::deque<GlobalPose> current_cruise_goal_deque_;

        GlobalPose current_x_y_; // 当前机器人的坐标点,来自nav2的footprint回调

        CruiseMode current_cruise_mode_ = CruiseMode::MY_HALF;  // 当前巡航模式，默认为MY_HALF
        CruiseMode previous_cruise_mode_ = CruiseMode::MY_HALF; // 记录上一次巡航模式，用于判断巡航模式是否切换

        std::chrono::steady_clock::time_point current_cruise_mode_start_time; // 上次使用当前巡航路径的时间点，用于达到阈值切换巡航路径，追击探测到目标后将current_cruise_mode_start_time重置为当前时间，主要防止单巡航路线长时间运行但是没有起作用

        const std::chrono::seconds CRUISE_MODE_DURATION_THRESHOLD_{30}; // 阈值：30秒

        GlobalPose human_goal_pose_; // 云台手发送的目标点

        /*********************3. 巡航相关变量 end  ******************************/

        /************************4.  追击相关变量 start ******************************/
        // 追击探测到目标后将current_cruise_mode_start_time重置为当前时间，主要防止单巡航路线长时间运行但是没有起作用
        /************************4. 追击相关变量 end ******************************/

        /************************5. 特殊区域相关变量 start ******************************/
        std::unordered_map<SpecialAreaType, std::vector<GlobalPose>> special_area_poses_;   // 特殊区域的所有边界点，通过名字获取
        std::unordered_map<std::string, std::vector<GlobalPose>> special_area_from_params_; // 从参数文件中读取的特殊区域的所有边界点，通过名字获取

        SpecialAreaType current_my_area_; // 当前所在区域

        SpecialAreaType target_area_; // 目标点所在区域

        std::unordered_map<SpecialAreaType, std::vector<GlobalPose>> upanddown_through_paths_; // 特殊区域的所有路径点，通过名字获取

        std::vector<GlobalPose> current_upanddown_goal_vector_; // 当前起伏路段的路径点

        bool is_current_fill_upanddown_goal_vector_; // 当前是否填充了起伏路段的路径点

        UpAndDownNavStatus upanddown_nav_status_ = UpAndDownNavStatus::NOT_START; // 起伏路段导航状态

        GlobalPose relocalization_point_; // 用于起伏路段重新定位的点, 一般是起伏结束的点
        /************************5. 特殊区域相关变量 end ******************************/

        /************************6. FSM状态相关变量 start ******************************/
        bool is_pose_from_human_ = false; // 标记是否接受到云台手发送的目标点

        bool is_need_supply_ = false; // 标记是否需要补给

        bool is_pursue_ = false; // 标记是否需要追击

        bool is_cruise_ = true; // 标记是否需要巡航，默认开启

        bool is_in_upanddown_area_ = false; // 是否在起伏路段区域的标志位

        /************************6. FSM状态相关变量 end ******************************/

        /*************************7. 姿态相关变量 start ******************************/
        enum class GestureType
        {
            // 暂时设置为追击：ATTACK，巡航：MOVE，回家补给：DEFEND
            ATTACK,
            MOVE,
            DEFEND
        };

        std::chrono::steady_clock::time_point current_gesture_start_time; // 上次使用当前姿态的时间点，用于达到阈值切换姿态

        const std::chrono::seconds GESTURE_DURATION_THRESHOLD_{30}; // 阈值：30秒
        /*************************7. 姿态相关变量 end ******************************/

        /**************************8. 线程和锁 start ******************************/

        std::mutex current_x_y_mutex_; // 当前机器人坐标点的互斥锁

        std::thread executor_thread_;               // 执行 callback_group_executor_ 的线程
        std::atomic<bool> executor_running_{false}; // executor 运行标志

        std::mutex fsm_state_mutex_;
        /**************************8. 线程和锁 end ******************************/


        /************************9. 云台手发送目标中断 start********************************/
        SpecialAreaType human_goal_area_; // 云台手发送目标点类型
        /************************9. 云台手发送目标中断 end********************************/

        /***************************10. 后视相机相关变量 start ******************************/
        /***************************10. 后视相机相关变量 end ******************************/

        /***************************11. 其他相关变量 start ******************************/
        std::chrono::steady_clock::time_point last_recv_time_; // 上次recv回调运行时间点，用于控制频率
        /***************************11. 其他相关变量 end ******************************/
    };
}