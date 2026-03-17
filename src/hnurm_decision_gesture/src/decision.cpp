#include "rclcpp/rclcpp.hpp"
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/utils/shared_library.h>
#include <behaviortree_cpp_v3/control_node.h>
#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/condition_node.h>

#include <GameStart.hpp>

#include <PubRobotStatus.hpp>
#include <filesystem>
#include <ament_index_cpp/get_package_share_directory.hpp>

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("bt_node");
    // std::string path = "/home/rm/slam/ws/src/hnu-vision-ros/hnurm_decision/src/decision.xml";
    std::string base_dir = ament_index_cpp::get_package_share_directory("hnurm_small_decision");

    std::string path = base_dir + "/param/tree.xml";

    BT::BehaviorTreeFactory factory; // 创建行为树工厂
    BT::SharedLibrary loader;        // 共享库加载器

    // 注册自定义节点
    factory.registerNodeType<hnurm_small_decision::GameStart>("GameStart");
    factory.registerNodeType<hnurm_small_decision::PubRobotStatus>("PubRobotStatus");

    // 加载Nav2的行为树节点插件
    factory.registerFromPlugin(loader.getOSName("nav2_is_stuck_condition_bt_node"));
    factory.registerFromPlugin(loader.getOSName("nav2_navigate_to_pose_action_bt_node"));
    factory.registerFromPlugin(loader.getOSName("nav2_wait_action_bt_node"));
    factory.registerFromPlugin(loader.getOSName("nav2_wait_cancel_bt_node"));
    factory.registerFromPlugin(loader.getOSName("nav2_goal_reached_condition_bt_node"));
    factory.registerFromPlugin(loader.getOSName("nav2_single_trigger_bt_node"));
    // 导航子树相关节点
    factory.registerFromPlugin(loader.getOSName("nav2_pipeline_sequence_bt_node"));
    factory.registerFromPlugin(loader.getOSName("nav2_rate_controller_bt_node"));
    factory.registerFromPlugin(loader.getOSName("nav2_follow_path_action_bt_node"));
    factory.registerFromPlugin(loader.getOSName("nav2_truncate_path_action_bt_node"));
    factory.registerFromPlugin(loader.getOSName("nav2_goal_updater_node_bt_node"));
    factory.registerFromPlugin(loader.getOSName("nav2_goal_updated_condition_bt_node"));
    factory.registerFromPlugin(loader.getOSName("nav2_compute_path_to_pose_action_bt_node"));

    factory.registerFromPlugin(loader.getOSName("nav2_compute_path_through_poses_action_bt_node"));
    // 清图相关节点
    factory.registerFromPlugin(loader.getOSName("nav2_clear_costmap_service_bt_node"));

    factory.registerFromPlugin(loader.getOSName("nav2_drive_on_heading_bt_node"));
    factory.registerFromPlugin(loader.getOSName("nav2_controller_selector_bt_node"));
    factory.registerFromPlugin(loader.getOSName("nav2_planner_selector_bt_node"));

    // 黑板配置
    BT::Blackboard::Ptr blackboard = BT::Blackboard::create();
    blackboard->set("node", node);                                                  // ROS 2节点实例，用于行为树节点中访问ROS功能
    blackboard->set("server_timeout", std::chrono::milliseconds(100000));           // 服务调用超时时间
    blackboard->set("bt_loop_duration", std::chrono::milliseconds(100));            // 行为树循环执行的时间间隔，行为树内存参数，存储在黑板中，可能被某些节点使用
    blackboard->set("wait_for_service_timeout", std::chrono::milliseconds(100000)); // 等待服务可用的超时时间
    auto tree = factory.createTreeFromFile(path, blackboard);

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    // 以一定频率持续运行行为树
    rclcpp::Rate loop_rate(50); // 50 Hz
    while (rclcpp::ok())
    {
        // rclcpp::spin_some(node);
        executor.spin_some(); // 处理所有待处理的ROS回调
        tree.tickRoot();      // tick 开始
        loop_rate.sleep();    // 本次tick周期结束，休眠以维持循环频率
    }
    rclcpp::shutdown();
    return 0;
}