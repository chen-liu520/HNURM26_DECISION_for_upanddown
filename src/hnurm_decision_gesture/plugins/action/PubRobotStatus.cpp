#include "hnurm_decision_gesture/PubRobotStatus.hpp"
#include <string>
#include "hnurm_interfaces/msg/gesture.hpp"
#include "behaviortree_cpp_v3/basic_types.h"

/*注释颜色：初始化：绿色，正常的：白色，错误的：红色，警告的：黄色*/
/*定时器监控所有状态：紫色35m*/
/*tick函数：蓝色34m*/
/*所有回调和调用的工具函数：青色36m*/
namespace hnurm_behavior_trees
{
    using std::placeholders::_1;

    PubRobotStatus::PubRobotStatus(
        const std::string &xml_tag_name,
        const BT::NodeConfiguration &conf)
        : BT::SyncActionNode(xml_tag_name, conf)
    {
        node_ = conf.blackboard->get<rclcpp::Node::SharedPtr>("node");
        /**************************回调组 start**************************************/
        callback_group_ = node_->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive,
            false);
        callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

        sub_option.callback_group = callback_group_;

        /**************************回调组 end**************************************/

        structure_get_params(); // 从参数服务器获取参数,填充相关变量

        structure_ROS_init();

        executor_running_ = true;
        executor_thread_ = std::thread([this]()
                                       {
            RCLCPP_INFO(node_->get_logger(), "[PubRobotStatus] callback_group_executor_ 线程启动");
            while (executor_running_) {
                callback_group_executor_.spin_some(std::chrono::milliseconds(10));
            }
            RCLCPP_INFO(node_->get_logger(), "[PubRobotStatus] callback_group_executor_ 线程退出"); });

        RCLCPP_INFO(node_->get_logger(), "PubRobotStatus::构造函数完成");
    }

    PubRobotStatus::~PubRobotStatus()
    {
        if (executor_thread_.joinable())
        {
            executor_thread_.join(); // 这个析构函数的主要作用是确保线程资源的正确回收
        }
        // 如果不调用 join()，线程可能在对象销毁后继续运行，导致以下问题：
        // 访问已释放的对象内存，引发未定义行为
        // 线程资源（如栈空间）无法被操作系统回收，造成内存泄漏
    }

    //  模式、状态等相关变量初始化
    void PubRobotStatus::initialization()
    {

        current_cruise_mode_ = CruiseMode::MY_HALF;                        // 初始化为默认巡航模式
        current_cruise_mode_start_time = std::chrono::steady_clock::now(); // 初始化巡航模式开始时间
        is_cruise_ = true;                                                 // 默认初始为巡航模式
        is_in_upanddown_area_ = false;                                     // 初始化起伏路段状态
        is_current_fill_upanddown_goal_vector_ = false;                    // 初始化起伏路段目标队列填充状态
        FillGoalsDeque_Cruise();

        if (self_color_ == SelfColor::RED)
        {
            structure_get_cruise_paths_from_paramsandcolor("red");
        }
        else
        {
            structure_get_cruise_paths_from_paramsandcolor("blue");
        }
        special_area_poses_[SpecialAreaType::MOVE_AREA] = special_area_from_params_["move_polygon"];
        special_area_poses_[SpecialAreaType::UPANDDOWN_AREA] = special_area_from_params_["upanddown_polygon"];

        is_init_ = true; // 初始化完成
    }

    void PubRobotStatus::structure_get_cruise_paths_from_paramsandcolor(std::string color)
    {
        const std::string GREEN = "\033[32m";
        const std::string RESET = "\033[0m";
        RCLCPP_INFO(node_->get_logger(), "[structure_get_cruise_paths] 开始读取%s方巡航路径", color.c_str());
        
        std::vector<std::string> cruise_goals_names = {
            "cruise_goals"};
        for (const auto &name : cruise_goals_names)
        {
            std::string param_name = color + "_goals_array." + name;
            node_->declare_parameter(param_name, std::vector<std::string>());
            auto goals_str = node_->get_parameter_or(param_name, std::vector<std::string>());
            
            RCLCPP_INFO(node_->get_logger(), "[structure_get_cruise_paths] 参数 %s: 读取到 %zu 个值", param_name.c_str(), goals_str.size());

            if (!goals_str.empty())
            {
                std::vector<GlobalPose> goals;
                for (const auto &point_str : goals_str)
                {
                    GlobalPose gp;
                    size_t comma_pos = point_str.find(',');
                    if (comma_pos != std::string::npos)
                    {
                        gp.pose_x = std::stof(point_str.substr(0, comma_pos));
                        gp.pose_y = std::stof(point_str.substr(comma_pos + 1));
                        goals.push_back(gp);
                    }
                }
                path_pose_arrays_[stringToCruiseMode(name)] = goals;
                RCLCPP_INFO(node_->get_logger(), "%s[structure_get_params] 加载 %s: %zu 个目标点%s",
                            GREEN.c_str(), name.c_str(), goals.size(), RESET.c_str());
            }
            else
            {
                RCLCPP_ERROR(node_->get_logger(), "%s[structure_get_params] %s is empty or not set%s",
                             GREEN.c_str(), name.c_str(), RESET.c_str());
            }
        }
    }

    /* @name: structure_ROS_init
     *  @brief：话题订阅，发布的初始化
     *  @param：
     */
    void PubRobotStatus::structure_ROS_init()
    {
        // 1. nav2的footprint回调订阅，获取当前机器人坐标点
        global_pose_sub_ = node_->create_subscription<geometry_msgs::msg::PolygonStamped>(
            global_position_topic_,
            rclcpp::SensorDataQoS(),
            std::bind(&PubRobotStatus::global_pose_callback, this, _1),
            sub_option);

        // 2.速度订阅
        cmd_vel_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
            cmd_vel_topic_,
            rclcpp::SensorDataQoS(),
            std::bind(&PubRobotStatus::remap_cmd_vel_callback, this, _1),
            sub_option);

        // 3. 订阅串口状态
        recv_sub_ = node_->create_subscription<hnurm_interfaces::msg::VisionRecvData>(
            recv_topic_,
            rclcpp::SensorDataQoS(),
            std::bind(&PubRobotStatus::recv_callback, this, _1),
            sub_option);

        // 4. 发布串口状态订阅，【更新目标信息】
        send_target_info_sub_ = node_->create_subscription<hnurm_interfaces::msg::VisionSendData>(
            send_topic_,
            rclcpp::ServicesQoS(),
            std::bind(&PubRobotStatus::send_target_info_callback, this, _1),
            sub_option);
        // x.速度发布
        cmd_vel_remap_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_remap_topic_, 10);

        // x. 发布巡航路径切换定时器
        special_area_timer_ = node_->create_wall_timer(std::chrono::milliseconds(223), std::bind(&PubRobotStatus::special_area_timer_callback, this), callback_group_);

        // x.
        change_yaw_angle_pub_ = node_->create_publisher<std_msgs::msg::Float32>(yaw_change_topic_, 10);

        // x. 进入起伏路段发布cotrol_id
        enter_upanddown_ControlID_pub_ = node_->create_publisher<std_msgs::msg::Float32>(enter_upanddown_ControlID_pub_topic_, 10);

        // xx.初始化tf2_ros::Buffer和tf2_ros::TransformListener
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    }

    /*  @name : structure_get_params
     *  @brief :读取参数文件，获取巡航、补给、特殊点位、特殊区域、特殊路径的所有路径点，通过名字获取
     *  @param :
     *  @return :
     */
    void PubRobotStatus::structure_get_params()
    {
        // 绿色ANSI颜色代码
        const std::string GREEN = "\033[32m";
        const std::string RESET = "\033[0m";

        RCLCPP_INFO(node_->get_logger(), "[structure_get_params] 开始读取参数，节点名称: %s", node_->get_name());

        // 0. 声明参数
        node_->declare_parameter("cmd_vel_topic", std::string("/cmd_vel"));
        node_->declare_parameter("global_position_topic", std::string("/global_costmap/published_footprint"));
        node_->declare_parameter("cmd_vel_remap_topic", std::string("/cmd_vel_remap"));
        node_->declare_parameter("gesture_pub_topic", std::string("/decision/gesture"));
        node_->declare_parameter("recv_topic", std::string("/vision_recv_data"));
        node_->declare_parameter("send_topic", std::string("/vision_send_data"));
        node_->declare_parameter("back_target_topic", std::string("/back_target"));
        node_->declare_parameter("human_goal_topic", std::string("/goal_pose"));
        node_->declare_parameter("yaw_change_topic", std::string("/decision/change_yaw_angle"));
        node_->declare_parameter("relocalization_topic", std::string("/relocalization"));
        node_->declare_parameter("enter_upanddown_ControlID_pub_topic", std::string("/decision/enter_upanddown_control_id"));
        node_->declare_parameter("relocalization_status_topic", std::string("/registration_status"));

        node_->declare_parameter("pursue_stay_circle_radius", 0.5); // 追击静止小陀螺的半径，默认0.5米

        // 1. 读取话题参数
        cmd_vel_topic_ = node_->get_parameter_or("cmd_vel_topic", std::string("/cmd_vel"));
        global_position_topic_ = node_->get_parameter_or("global_position_topic", std::string("/global_costmap/published_footprint"));
        cmd_vel_remap_topic_ = node_->get_parameter_or("cmd_vel_remap_topic", std::string("/cmd_vel_remap"));
        gesture_pub_topic_ = node_->get_parameter_or("gesture_pub_topic", std::string("/decision/gesture"));
        recv_topic_ = node_->get_parameter_or("recv_topic", std::string("/vision_recv_data"));
        send_topic_ = node_->get_parameter_or("send_topic", std::string("/vision_send_data"));
        back_target_topic_ = node_->get_parameter_or("back_target_topic", std::string("/back_target"));
        human_goal_topic_ = node_->get_parameter_or("human_goal_topic", std::string("/goal_pose"));
        yaw_change_topic_ = node_->get_parameter_or("yaw_change_topic", std::string("/decision/change_yaw_angle"));
        relocalization_topic_ = node_->get_parameter_or("relocalization_topic", std::string("/relocalization"));
        enter_upanddown_ControlID_pub_topic_ = node_->get_parameter_or("enter_upanddown_ControlID_pub_topic", std::string("/decision/enter_upanddown_control_id"));
        relocalization_status_topic_ = node_->get_parameter_or("relocalization_status_topic", std::string("/registration_status"));


        RCLCPP_INFO(node_->get_logger(), "%s[structure_get_params] cmd_vel_topic: %s%s",
                    GREEN.c_str(), cmd_vel_topic_.c_str(), RESET.c_str());
        RCLCPP_INFO(node_->get_logger(), "%s[structure_get_params] global_position_topic: %s%s",
                    GREEN.c_str(), global_position_topic_.c_str(), RESET.c_str());
        RCLCPP_INFO(node_->get_logger(), "%s[structure_get_params] cmd_vel_remap_topic: %s%s",
                    GREEN.c_str(), cmd_vel_remap_topic_.c_str(), RESET.c_str());
        RCLCPP_INFO(node_->get_logger(), "%s[structure_get_params] gesture_pub_topic: %s%s",
                    GREEN.c_str(), gesture_pub_topic_.c_str(), RESET.c_str());
        RCLCPP_INFO(node_->get_logger(), "%s[structure_get_params] recv_topic: %s%s",
                    GREEN.c_str(), recv_topic_.c_str(), RESET.c_str());
        RCLCPP_INFO(node_->get_logger(), "%s[structure_get_params] send_topic: %s%s",
                    GREEN.c_str(), send_topic_.c_str(), RESET.c_str());
        RCLCPP_INFO(node_->get_logger(), "%s[structure_get_params] back_target_topic: %s%s",
                    GREEN.c_str(), back_target_topic_.c_str(), RESET.c_str());
        RCLCPP_INFO(node_->get_logger(), "%s[structure_get_params] human_goal_topic: %s%s",
                    GREEN.c_str(), human_goal_topic_.c_str(), RESET.c_str());
        RCLCPP_INFO(node_->get_logger(), "%s[structure_get_params] yaw_change_topic: %s%s",
                    GREEN.c_str(), yaw_change_topic_.c_str(), RESET.c_str());
        RCLCPP_INFO(node_->get_logger(), "%s[structure_get_params] relocalization_topic: %s%s",
                    GREEN.c_str(), relocalization_topic_.c_str(), RESET.c_str());
        RCLCPP_INFO(node_->get_logger(), "%s[structure_get_params] relocalization_status_topic: %s%s",
                    GREEN.c_str(), relocalization_status_topic_.c_str(), RESET.c_str());
        RCLCPP_INFO(node_->get_logger(), "%s[structure_get_params] enter_upanddown_ControlID_pub_topic: %s%s",
                    GREEN.c_str(), enter_upanddown_ControlID_pub_topic_.c_str(), RESET.c_str());

        // 2. 读取特殊区域相关参数 (在params的special_area空间下)
        std::vector<std::string> special_area_names = {
            "upanddown_polygon", "move_polygon"};
        for (const auto &name : special_area_names)
        {
            std::string param_name = "special_area." + name;
            node_->declare_parameter(param_name, std::vector<std::string>());
            auto goals_str = node_->get_parameter_or(param_name, std::vector<std::string>());

            if (!goals_str.empty())
            {
                std::vector<GlobalPose> goals;
                for (const auto &point_str : goals_str)
                {
                    GlobalPose gp;
                    size_t comma_pos = point_str.find(',');
                    if (comma_pos != std::string::npos)
                    {
                        gp.pose_x = std::stof(point_str.substr(0, comma_pos));
                        gp.pose_y = std::stof(point_str.substr(comma_pos + 1));
                        goals.push_back(gp);
                    }
                }
                special_area_from_params_[name] = goals;
                RCLCPP_INFO(node_->get_logger(), "%s[structure_get_params] Loaded %s: %zu points%s",
                            GREEN.c_str(), name.c_str(), goals.size(), RESET.c_str());
            }
            else
            {
                RCLCPP_ERROR(node_->get_logger(), "%s[structure_get_params] %s is empty or not set%s",
                             GREEN.c_str(), name.c_str(), RESET.c_str());
            }
        }

        // 3. 读取起伏路段区域内穿越路径 (upanddown_path)
        // 红色方起伏路段穿越路径
        node_->declare_parameter("special_area.upanddown_path", std::vector<std::string>());
        auto red_upanddown_path = node_->get_parameter_or(
            "special_area.upanddown_path", std::vector<std::string>());
        
        RCLCPP_INFO(node_->get_logger(), "[structure_get_params] 参数 special_area.upanddown_path: 读取到 %zu 个值", red_upanddown_path.size());
        
        if (!red_upanddown_path.empty())
        {
            std::vector<GlobalPose> path_points;
            for (const auto &point_str : red_upanddown_path)
            {
                GlobalPose gp;
                size_t comma_pos = point_str.find(',');
                if (comma_pos != std::string::npos)
                {
                    gp.pose_x = std::stof(point_str.substr(0, comma_pos));
                    gp.pose_y = std::stof(point_str.substr(comma_pos + 1));
                    path_points.push_back(gp);
                }
            }
            upanddown_through_paths_[SpecialAreaType::UPANDDOWN_AREA] = path_points;
            RCLCPP_INFO(node_->get_logger(), "%s[structure_get_params] Loaded UPANDDOWN_AREA path: %zu points%s",
                        GREEN.c_str(), path_points.size(), RESET.c_str());
        }
        else
        {
            RCLCPP_ERROR(node_->get_logger(), "%s[structure_get_params] red_upanddown_path is empty or not set%s",
                         GREEN.c_str(), RESET.c_str());
        }
    }

    BT::NodeStatus PubRobotStatus::tick()
    {
        if (!is_init_)
        {
            return BT::NodeStatus::FAILURE;
        }

        std::unique_lock<std::mutex> lock(fsm_state_mutex_);

        // 1. 检查导航目标是否到达，如果到达则更新目标点队列
        CheckIsReached_Update(0.2); // 内部不再加锁
        setOutput("current_controller", "Omni");

        // 2. 根据状态机逻辑设置黑板变量
        //******************待尝试，直接用，不去用有限状态机去更新 *****************/
        // setOutput("need_supply", need_supply_);
        // setOutput("pose_from_human", is_pose_from_human_);
        // setOutput("is_pursue", is_pursue_);
        // setOutput("is_cruise", is_cruise_);
        // setOutput("is_in_upanddown_area", is_in_upanddown_area_);
        if (is_pose_from_human_)
        {
            FillGoalsDeque_Cruise();
            setOutput("is_in_upanddown_area", false);
            setOutput("pose_from_human", true);
            setOutput("need_supply", false);
            setOutput("is_pursue", false);
            setOutput("is_cruise", false);
            setOutput("human_goal", GlobalPose2PoseStamped(false));

            return BT::NodeStatus::SUCCESS;
        }
        if (is_need_supply_)
        {
            if (previous_cruise_mode_ != current_cruise_mode_)
            {
                // 刚切换到补给模式，重新装填
                FillGoalsDeque_Cruise();
                RCLCPP_INFO(node_->get_logger(), "\033[34m[状态切换] 切换到补给模式\033[0m");
            }
            setOutput("need_supply", true);
            setOutput("pose_from_human", false);
            setOutput("is_pursue", false);
            setOutput("is_cruise", false);
            setOutput("supply_goal", GlobalPose2PoseStamped(false));

            return BT::NodeStatus::SUCCESS;
        }
        if (is_pursue_)
        {
            setOutput("is_pursue", true);
            setOutput("pose_from_human", false);
            setOutput("need_supply", false);
            setOutput("is_cruise", false);
            setOutput("pursue_goal", GlobalPose2PoseStamped(true));

            return BT::NodeStatus::SUCCESS;
        }
        if (is_cruise_)
        {
            setOutput("is_cruise", true);
            setOutput("pose_from_human", false);
            setOutput("need_supply", false);
            setOutput("is_pursue", false);
            setOutput("cruise_goal", GlobalPose2PoseStamped(false));

            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::FAILURE;
    }

    /***********************************************工具函数 start *********************************************************/

    /*  @name : CheckIsReached_Update
     *  @brief : 检查是否到达导航点，并更新目标点队列
     *  @param : current_pose 当前机器人位姿(来自订阅的导航话题)，current_goal_pose 当前导航点（来自队列top（））, threshold 到达阈值
     *  @return : bool 是否到达导航点
     */
    void PubRobotStatus::CheckIsReached_Update(float threshold)
    {
        if (current_cruise_goal_deque_.empty())
        {
            RCLCPP_ERROR(node_->get_logger(), "[CheckIsReached_Update] 巡航目标点队列为空，无法检查是否到达目标点");
            FillGoalsDeque_Cruise(); // 尝试重新装填巡航目标点队列
            return;
        }
        GlobalPose current_goal_pose = current_cruise_goal_deque_.front();    // 获取当前目标点
        GlobalPose current_pose = current_x_y_;                               // 获取当前机器人坐标点，来自global_pose_callback
        float distance = calculate_distance(current_pose, current_goal_pose); // 计算当前坐标点与目标点的距离
        // 注意：虽然巡航，云台打断，补给公用一个逻辑，但是if的顺序就是决策树优先级：云台打断 > 补给 > 巡航，保证状态优先级正确
        if (is_pose_from_human_)
        {
            if (distance <= threshold) // 到达目标点
            {
                RCLCPP_INFO(node_->get_logger(), "\033[34m[CheckIsReached_Update] 已到达云台手目标点\033[0m");
            }
        }
        else if (is_need_supply_)
        {
            if (distance <= threshold) // 到达补给点
            {
                RCLCPP_INFO(node_->get_logger(), "\033[34m[CheckIsReached_Update] 已到达补给点\033[0m");
                previous_cruise_mode_ = current_cruise_mode_;      // 记录上一个巡航模式
                current_cruise_mode_ = CruiseMode::HIGHLAND_MIXED;
            }
        }
        else if (is_cruise_)
        {
            if (distance <= threshold && current_cruise_mode_ != CruiseMode::SUPPLY && current_cruise_mode_ != CruiseMode::MY_FORT) // 到达目标点且不是补给点或堡垒点
            {
                current_cruise_goal_deque_.pop_front(); // 从队列中移除已到达的目标点
            }
        }
        else
        {
        }
    }

    void PubRobotStatus::FillGoalsDeque_Cruise()
    {
        // mode ：【human,cruise_goals_supply,cruise_goals_my_fort,cruise_goals_my_fort, cruise_goals_my_half,cruise_goals_opponent_half,cruise_goals_highland_mixed, cruise_goals_highland_ambush】
        current_cruise_goal_deque_.clear();

        if (current_cruise_mode_ == CruiseMode::MY_HALF) // 补给模式只装填补给点
        {
            for (int i = 0; i < 2; ++i)
            {
                current_cruise_goal_deque_.push_back(human_goal_pose_);
            }
            current_cruise_goal_deque_.pop_back();
        }
    }

    void PubRobotStatus::FillGoalsVector_UpAndDown()
    {
        if (is_current_fill_upanddown_goal_vector_)
            return;
        
        // 检查路径是否存在且至少有2个点
        auto& path = upanddown_through_paths_[SpecialAreaType::UPANDDOWN_AREA];
        if (path.size() < 2)
        {
            RCLCPP_WARN(node_->get_logger(), "\033[36m[FillGoalsVector_UpAndDown] 起伏路段路径点数量不足(%zu < 2)，无法填充目标队列\033[0m", path.size());
            is_current_fill_upanddown_goal_vector_ = true;
            return;
        }
        
        if (current_my_area_ == SpecialAreaType::UPANDDOWN_AREA)
        {
            if (calculate_distance(current_x_y_, path.front()) < calculate_distance(current_x_y_, path.back()))
            {
                for (size_t i = 1; i < path.size(); ++i)
                {
                    current_upanddown_goal_vector_.push_back(path[i]);
                }
            }
            else
            {
                // 使用有符号整数避免下溢
                for (int i = static_cast<int>(path.size()) - 2; i >= 0; --i)
                {
                    current_upanddown_goal_vector_.push_back(path[i]);
                }
            }
        }

        is_current_fill_upanddown_goal_vector_ = true;
        // 检查队列大小，避免越界访问
        if (current_upanddown_goal_vector_.size() >= 2)
        {
            relocalization_point_ = current_upanddown_goal_vector_[1];
        }
        else if (!current_upanddown_goal_vector_.empty())
        {
            relocalization_point_ = current_upanddown_goal_vector_.back();
        }
        RCLCPP_INFO(node_->get_logger(), "\033[36m[FillGoalsVector_UpAndDown] 已填充起伏路段目标点队列，当前队列长度: %zu\033[0m", current_upanddown_goal_vector_.size());
    }

    void PubRobotStatus::UpdateCurrentCruiseMode()
    {
        // 1. 保存上一次巡航模式
        previous_cruise_mode_ = current_cruise_mode_;

        // 2. 直接返回，保护补给和人类输入模式不被其他逻辑覆盖，直到需要补给和人类输入的状态消失
        if (current_cruise_mode_ == CruiseMode::SUPPLY || current_cruise_mode_ == CruiseMode::HUMAN)
        {
            return;
        }
    }

    /* @调用频率 : 10ms (100Hz，由remap_cmd_vel_callback调用)
     * @执行时间 :
     *   - RELOCATING状态: ~30ns (直接返回)
     *   - 正常导航(TF缓存命中): ~0.5-1ms
     *   - TF查询等待: ~10ms (最坏情况，超时)
     * @瓶颈 : TF查询 (lookupTransform)
     */
    geometry_msgs::msg::Twist PubRobotStatus::NavigateInUpAndDownArea()
    {
        geometry_msgs::msg::Twist cmd_vel;
        cmd_vel.linear.x = 0.0; // 显式初始化
        cmd_vel.linear.y = 0.0;
        cmd_vel.linear.z = 0.0;
        cmd_vel.angular.x = 0.0;
        cmd_vel.angular.y = 0.0;
        cmd_vel.angular.z = recv_robot_information_.yaw;

        if (current_upanddown_goal_vector_.empty())
        {
            upanddown_nav_status_ = UpAndDownNavStatus::NOT_START;
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 500,
                                 "\033[36m[NavigateInUpAndDownArea] 起伏路段目标点队列为空\033[0m");
            return cmd_vel;
        }

        geometry_msgs::msg::TransformStamped map_to_base_footprint;
        try
        {
            // 使用50ms前的TF数据，平衡实时性和查询速度
            rclcpp::Time target_time = node_->now() - rclcpp::Duration::from_seconds(0.05);
            map_to_base_footprint = tf_buffer_->lookupTransform(
                "map",
                "base_footprint",
                target_time,
                rclcpp::Duration::from_seconds(0.01));
        }
        catch (const tf2::TransformException &e)
        {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 500,
                                 "\033[36m[NavigateInUpAndDownArea] TF查询失败: %s\033[0m", e.what());
            return cmd_vel;
        }

        tf2::Quaternion q(map_to_base_footprint.transform.rotation.x,
                          map_to_base_footprint.transform.rotation.y,
                          map_to_base_footprint.transform.rotation.z,
                          map_to_base_footprint.transform.rotation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw); // 相对于地图坐标系的航向角，范围是[-pi, pi]，逆时针为正，顺时针为负

        GlobalPose current_pose;
        {
            std::lock_guard<std::mutex> lock(current_x_y_mutex_);
            current_pose = current_x_y_;
        }

        const GlobalPose &target = current_upanddown_goal_vector_.front();
        const double dx = static_cast<double>(target.pose_x - current_pose.pose_x);
        const double dy = static_cast<double>(target.pose_y - current_pose.pose_y);
        const double dist = std::hypot(dx, dy); // sqrt(dx² + dy²)

        const double yaw_target = std::atan2(dy, dx); // 以当前位置为原点，从x轴正方向逆时针到目标点的角度范围是[-pi, pi]，逆时针为正，顺时针为负
        // tar-now，虽然lookup和atan2的范围都是[-pi, pi]，但tar-now的范围是[0, 2pi]，需要注意这个角度差值的计算，不能直接相减，因为可能出现跨越±pi的情况，需要用atan2来计算正确的角度差值，保证结果在[-pi, pi]范围内
        // 归一化到 [-π, π] 范围，这是atan2的特性：返回 θ 在 [-π, π] 范围内的等效角度
        const double yaw_error = std::atan2(std::sin(yaw_target - yaw), std::cos(yaw_target - yaw));

        constexpr double goal_reached_dist = 0.10; // 到达目标点的距离阈值
        constexpr double spin_done_yaw_err = 0.10; // 自旋完成的角度误差阈值
        constexpr double re_spin_yaw_err = 0.15;    // 重新进入自旋的角度误差阈值
        //constexpr double k_spin = 1.0;             // 自旋控制增益
        //constexpr double k_drive_w = 0.8;          // 直线行驶时的角速度控制增益
        //constexpr double w_spin_max = 1.2;         // 自旋最大角速度
        //constexpr double w_drive_max = 0.8;        // 直线行使时最大角速度
        constexpr double v_max = 0.6;              // 线速度最大速度

        if (dist < goal_reached_dist)
        {
            current_upanddown_goal_vector_.erase(current_upanddown_goal_vector_.begin());
            upanddown_nav_status_ = UpAndDownNavStatus::NOT_START;
            RCLCPP_INFO(node_->get_logger(), "\033[36m[NavigateInUpAndDownArea] 已到达起伏路段目标点，剩余: %zu\033[0m",
                        current_upanddown_goal_vector_.size());

            if (current_upanddown_goal_vector_.empty())
            {
                RCLCPP_INFO(node_->get_logger(), "\033[36m[NavigateInUpAndDownArea] 起伏路段路径执行完成\033[0m");
            }
            return cmd_vel;
        }

        if (upanddown_nav_status_ == UpAndDownNavStatus::NOT_START)
        {
            upanddown_nav_status_ = UpAndDownNavStatus::SPIN;
        }

        // 将弧度误差转换为角度误差（电控需要绝对角度，单位为度）
        constexpr double RAD_TO_DEG = 180.0 / 3.14159265358979323846;
        double yaw_error_deg = yaw_error * RAD_TO_DEG;

        if (upanddown_nav_status_ == UpAndDownNavStatus::SPIN)
        {
            // 计算目标绝对角度 = 当前绝对角度 + 角度误差
            cmd_vel.angular.z = recv_robot_information_.yaw + yaw_error_deg;
            if (std::fabs(yaw_error) < spin_done_yaw_err)
            {
                upanddown_nav_status_ = UpAndDownNavStatus::LINEAR;
            }
            return cmd_vel;
        }

        if (std::fabs(yaw_error) > re_spin_yaw_err)
        {
            upanddown_nav_status_ = UpAndDownNavStatus::SPIN;
            cmd_vel.angular.z = recv_robot_information_.yaw + yaw_error_deg;
            return cmd_vel;
        }

        // 小于3.5米时，速度为0.5*距离，范围减小；反之，速度为0.9*距离，范围增大，但都限制在合理范围内，避免过快或过慢
        const double v_ff = (dist < 0.2) ? std::clamp(0.5 * dist, 0.10, 0.25) : std::clamp(0.9 * dist, 0.10, v_max);
        cmd_vel.linear.x = v_ff;
        cmd_vel.linear.y = 0.0;
        // LINEAR模式下计算目标绝对角度
        if(std::abs(yaw_error_deg) < 10.0)
            yaw_error_deg = 0.0; // 当角度误差较小时，认为不需要调整角度，避免震荡
        cmd_vel.angular.z = recv_robot_information_.yaw + yaw_error_deg;
        return cmd_vel;
    }

    PubRobotStatus::SpecialAreaType PubRobotStatus::GetPointArea(const GlobalPose &point)
    {
        // 注意这里的顺序，为了避免有洞区域的复杂判断，对洞内区域(双方堡垒)先判断，是的话直接返回，不是的话才会判断是不是属于其他区域（包括套着他的区域（双方半场））
        // 顺序：1. 双方堡垒 2. 双方起伏路段 3. 高地 4. 双方半场
        std::vector<SpecialAreaType> check_order = {
            SpecialAreaType::UPANDDOWN_AREA,
            SpecialAreaType::MOVE_AREA};
        for (auto type : check_order)
        {
            if (is_in_this_polygon(point, special_area_poses_[type]))
            {
                return type;
            }
        }
        return SpecialAreaType::ILLEGAL_AREA;
    }

    /* @name : GlobalPose2PoseStamped
     *  @brief : 将全局坐标点转换为PoseStamped消息
     *  @param : global_pose 全局坐标点
     *  @return : pose_stamped PoseStamped消息
     */
    geometry_msgs::msg::PoseStamped PubRobotStatus::GlobalPose2PoseStamped(bool is_pursue)
    {
        GlobalPose global_pose;
        if (is_pursue)
        {
            {
                std::lock_guard<std::mutex> lock(current_x_y_mutex_);
                global_pose = current_x_y_; // 使用当前位置作为备选
            }
        }
        else
        {
            if (current_cruise_goal_deque_.empty())
            {
                RCLCPP_ERROR(node_->get_logger(), "[tick] Cruise goal deque is empty!");
                {
                    std::lock_guard<std::mutex> lock(current_x_y_mutex_);
                    global_pose = current_x_y_; // 使用当前位置作为备选
                }
            }
            else
            {
                global_pose = current_cruise_goal_deque_.front(); // 获取当前巡航目标点
            }
        }
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header.stamp = node_->now();
        pose_stamped.header.frame_id = "map";
        pose_stamped.pose.position.x = global_pose.pose_x;
        pose_stamped.pose.position.y = global_pose.pose_y;
        pose_stamped.pose.position.z = 0.0;
        pose_stamped.pose.orientation.x = 0.0;
        pose_stamped.pose.orientation.y = 0.0;
        pose_stamped.pose.orientation.z = 0.0;
        pose_stamped.pose.orientation.w = 1.0;
        return pose_stamped;
    }

    /* @name: calculateAverage
     *  @brief：用于计算多边形点集的平均坐标，作为机器人当前位置的估计值。将定位系统发布的 footprint（多边形）转换为机器人的中心点坐标
     *  @param：msg geometry_msgs::msg::PolygonStamped msg - 包含多边形点集的 ROS 消息
     */
    PubRobotStatus::GlobalPose PubRobotStatus::calculateAverage(geometry_msgs::msg::PolygonStamped &msg)
    {
        double x_sum = 0.0;
        double y_sum = 0.0;
        GlobalPose cgp_;
        for (auto const &p : msg.polygon.points)
        {
            x_sum += p.x;
            y_sum += p.y;
        }
        cgp_.pose_x = x_sum / msg.polygon.points.size();
        cgp_.pose_y = y_sum / msg.polygon.points.size();
        return cgp_;
    }

    bool PubRobotStatus::is_in_this_polygon(const GlobalPose &point, const std::vector<GlobalPose> &polygon)
    {
        const float EPSILON = 1e-9;
        bool inside = false;
        int n = polygon.size();

        for (int i = 0, j = n - 1; i < n; j = i++)
        {
            const auto &pi = polygon[i];
            const auto &pj = polygon[j];

            // 射线法核心判断
            if (((pi.pose_y > point.pose_y) != (pj.pose_y > point.pose_y)) &&
                (point.pose_x < (pj.pose_x - pi.pose_x) * (point.pose_y - pi.pose_y) / (pj.pose_y - pi.pose_y + EPSILON) + pi.pose_x))
            {
                inside = !inside;
            }
        }
        return inside;
    }

    float PubRobotStatus::calculate_distance(const GlobalPose &pose1, const GlobalPose &pose2)
    {
        return std::sqrt(std::pow(pose1.pose_x - pose2.pose_x, 2) + std::pow(pose1.pose_y - pose2.pose_y, 2));
    }

    PubRobotStatus::CruiseMode PubRobotStatus::stringToCruiseMode(const std::string &mode_str)
    {
        if (mode_str == "human")
            return CruiseMode::HUMAN;
        if (mode_str == "cruise_goals")
            return CruiseMode::MY_HALF; // 默认巡航模式是我方半场，可以根据需要调整
        if (mode_str == "cruise_goals_supply")
            return CruiseMode::SUPPLY;
        if (mode_str == "cruise_goals_my_fort")
            return CruiseMode::MY_FORT;
        if (mode_str == "cruise_goals_opponent_fort")
            return CruiseMode::OPPONENT_FORT;
        if (mode_str == "cruise_goals_my_half")
            return CruiseMode::MY_HALF;
        if (mode_str == "cruise_goals_opponent_half")
            return CruiseMode::OPPONENT_HALF;

        if (mode_str == "cruise_goals_highland")
            return CruiseMode::HIGHLAND;
        if (mode_str == "cruise_goals_highland_ambush")
            return CruiseMode::HIGHLAND_AMBUSH;
        if (mode_str == "cruise_goals_highland_mixed")
            return CruiseMode::HIGHLAND_MIXED;

        return CruiseMode::MY_HALF; // 默认返回
    }

    std::string PubRobotStatus::cruiseModeToString(CruiseMode mode)
    {
        switch (mode)
        {
        case CruiseMode::HUMAN:
            return "human";
        case CruiseMode::SUPPLY:
            return "cruise_goals_supply";
        case CruiseMode::MY_FORT:
            return "cruise_goals_my_fort";
        case CruiseMode::OPPONENT_FORT:
            return "cruise_goals_opponent_fort";
        case CruiseMode::MY_HALF:
            return "moving";
        case CruiseMode::HIGHLAND:
            return "cruise_goals_highland";
        case CruiseMode::OPPONENT_HALF:
            return "cruise_goals_opponent_half";
        case CruiseMode::HIGHLAND_AMBUSH:
            return "cruise_goals_highland_ambush";
        case CruiseMode::HIGHLAND_MIXED:
            return "cruise_goals_highland_mixed";
        }
        return "unknown";
    }

    std::string PubRobotStatus::upanddownNavStatusToString(const UpAndDownNavStatus &status)
    {
        switch (status)
        {
        case UpAndDownNavStatus::NOT_START:
            return "NOT_START";
        case UpAndDownNavStatus::SPIN:
            return "SPIN";
        case UpAndDownNavStatus::RELOCATING:
            return "RELOCATING";
        case UpAndDownNavStatus::LINEAR:
            return "LINEAR";
        }
        return "UNKNOWN";
    }
    /***********************************************工具函数 end *********************************************************/

    /**************************回调函数 start*********************************/

    /* @name : global_pose_callback 10ms（不确定需要测试）
     * @brief : 全局定位回调函数,填充现在机器人坐标
     * @param : msg 全局定位消息
     * @return : None
     * @周期 : ~20ms (50Hz，由全局定位系统发布频率决定)
     * @执行时间 : ~1-5μs (计算平均值+区域检测)
     */
    void PubRobotStatus::global_pose_callback(const geometry_msgs::msg::PolygonStamped::SharedPtr msg)
    {
        if (!is_init_)
            return;
        std::lock_guard<std::mutex> lock(current_x_y_mutex_);
        current_x_y_ = calculateAverage(*msg);
        current_my_area_ = GetPointArea(current_x_y_);
    }

    /* 100Hz
     * @name : remap_cmd_vel_callback
     * @brief : 速度命令回调重映射函数,起伏路段自控制，后时旋转180,其他情况不变
     * @param : msg 速度命令消息
     * @周期 : 10ms (100Hz)
     * @执行时间 : ~1-5μs (普通情况) / ~0.5-10ms (起伏路段，含TF查询)
     */
    void PubRobotStatus::remap_cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        geometry_msgs::msg::Twist vel_pub_;
        vel_pub_ = *msg;

        if (is_in_upanddown_area_)
        {
            vel_pub_ = NavigateInUpAndDownArea();
        }
        cmd_vel_remap_pub_->publish(vel_pub_);
    }

    /* @周期 : 1ms(串口1000Hz，但我们只处理50Hz的消息，过快的消息会被节流掉)
     * @执行时间 : ~1μs (简单状态判断)
     */
    void PubRobotStatus::recv_callback(const hnurm_interfaces::msg::VisionRecvData::SharedPtr msg)
    {
        // 处理串口状态消息，更新内部状态变量
        recv_robot_information_ = *msg;
        if (recv_robot_information_.self_color.data != 0)
        {
            self_color_ = recv_robot_information_.self_color.data == 1 ? SelfColor::RED : SelfColor::BLUE;
            is_color_setted_ = true; // 颜色信息已设置
        }
    }

    /* @周期 : 20ms
     * @执行时间 : ~1-5μs (状态切换逻辑)
     */
    void PubRobotStatus::send_target_info_callback(const hnurm_interfaces::msg::VisionSendData::SharedPtr msg)
    {
        if (!is_init_)
            return;
        // 处理视觉系统发送的目标状态信息，控制机器人行为模式切换
        send_target_information_ = *msg;
    }

    /* @周期 : 223ms (5Hz)
     * @执行时间 : ~50-100μs (区域检测+距离计算+日志输出)
     * @说明 : 可能是最耗时的回调，包含大量日志输出
     */
    void PubRobotStatus::special_area_timer_callback()
    {
        {
            std::lock_guard<std::mutex> lock(fsm_state_mutex_);

            if (is_color_setted_ && !is_init_)
            {
                initialization(); // 省一个定时器，就在这里循环判断初始化
            }
            if (current_my_area_ == SpecialAreaType::UPANDDOWN_AREA)
            {
                if (!is_in_upanddown_area_)
                {
                    is_in_upanddown_area_ = true;
                    FillGoalsVector_UpAndDown();
                    RCLCPP_INFO(node_->get_logger(), "\033[36m[状态切换] 进入起伏路段，切换到起伏巡航模式\033[0m");
                }
            }
            else
            {
                is_current_fill_upanddown_goal_vector_ = false;
                is_in_upanddown_area_ = false;
                upanddown_nav_status_ = UpAndDownNavStatus::NOT_START;
            }
        }

        if (is_in_upanddown_area_)
        {
            enter_upanddown_ControlID_pub_->publish(std_msgs::msg::Float32().set__data(25.0)); // 进入起伏路段时发布25.0，下位机切换底盘跟随模式
        }
        else
        {
            enter_upanddown_ControlID_pub_->publish(std_msgs::msg::Float32().set__data(50.0));
        }
        RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 500, "\033[35m----------------------------------------------------------------\033[0m");
        RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 500, "\033[35m【定时器状态和数据打印】：当前总状态：%s，当前巡航模式：%s，当前是否在起伏路段：%s\033[0m",
                             is_pose_from_human_ ? "人类打断" : (is_need_supply_ ? "补给" : (is_pursue_ ? "追击" : (is_cruise_ ? "巡航" : "其他"))),
                             cruiseModeToString(current_cruise_mode_).c_str(), is_in_upanddown_area_ ? "是" : "否");
        RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 500, "\033[35m当前坐标: (%.2f, %.2f)，当前区域: %s\033[0m",
                             current_x_y_.pose_x, current_x_y_.pose_y,
                             current_my_area_ == SpecialAreaType::UPANDDOWN_AREA ? "起伏路段" : (current_my_area_ == SpecialAreaType::MOVE_AREA ? "移动区域" : "非法区域"));
        RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 500, "\033[35m当前起伏路段状态：%s\033[0m", upanddownNavStatusToString(upanddown_nav_status_).c_str());
        RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 500, "\033[35m----------------------------------------------------------------\033[0m");
    }

    /**************************回调函数 end*********************************/
}

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<hnurm_behavior_trees::PubRobotStatus>("PubRobotStatus");
}