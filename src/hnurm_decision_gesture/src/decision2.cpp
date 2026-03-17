/**
 * @file decision2.cpp
 * @brief 统一线程池方案的决策节点实现
 * 
 * 架构特点：
 * 1. 单一MultiThreadedExecutor处理所有ROS回调（线程池）
 * 2. BT树tick在主线程以固定频率执行，与ROS回调隔离
 * 3. 回调按频率分组管理，避免资源浪费
 */

#include "rclcpp/rclcpp.hpp"
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/utils/shared_library.h>
#include <behaviortree_cpp_v3/control_node.h>
#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/condition_node.h>

#include <ExampleUnifiedNode.hpp>
#include <filesystem>
#include <ament_index_cpp/get_package_share_directory.hpp>

/**
 * @brief 回调组管理器
 * 统一管理所有回调组，按频率分类，避免每个BT节点创建独立线程
 */
class CallbackGroupManager
{
public:
    /**
     * @brief 频率级别枚举，用于分类回调组
     */
    enum class FrequencyLevel
    {
        HIGH,   // 高频：>100Hz，如IMU、里程计
        MEDIUM, // 中频：10-100Hz，如视觉、控制
        LOW,    // 低频：<10Hz，如裁判系统、配置
        MUTEX   // 互斥：需要串行执行的关键业务
    };

    /**
     * @brief 初始化回调组管理器
     * @param node ROS2节点指针
     */
    void init(rclcpp::Node::SharedPtr node)
    {
        node_ = node;
        
        // 创建不同频率的回调组
        // Reentrant类型：允许同一组内回调并行执行，提高吞吐量
        high_freq_group_ = node_->create_callback_group(
            rclcpp::CallbackGroupType::Reentrant);
        
        medium_freq_group_ = node_->create_callback_group(
            rclcpp::CallbackGroupType::Reentrant);
        
        low_freq_group_ = node_->create_callback_group(
            rclcpp::CallbackGroupType::Reentrant);
        
        // MutuallyExclusive类型：同一组内回调串行执行，保证数据一致性
        mutex_group_ = node_->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);

        RCLCPP_INFO(node_->get_logger(), 
            "CallbackGroupManager initialized with 4 groups");
    }

    /**
     * @brief 获取指定频率级别的回调组
     */
    rclcpp::CallbackGroup::SharedPtr get_group(FrequencyLevel level)
    {
        switch (level)
        {
            case FrequencyLevel::HIGH:   return high_freq_group_;
            case FrequencyLevel::MEDIUM: return medium_freq_group_;
            case FrequencyLevel::LOW:    return low_freq_group_;
            case FrequencyLevel::MUTEX:  return mutex_group_;
            default: return medium_freq_group_;
        }
    }

    // 便捷的获取方法
    rclcpp::CallbackGroup::SharedPtr high_freq()   { return high_freq_group_; }
    rclcpp::CallbackGroup::SharedPtr medium_freq() { return medium_freq_group_; }
    rclcpp::CallbackGroup::SharedPtr low_freq()    { return low_freq_group_; }
    rclcpp::CallbackGroup::SharedPtr mutex()       { return mutex_group_; }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::CallbackGroup::SharedPtr high_freq_group_;
    rclcpp::CallbackGroup::SharedPtr medium_freq_group_;
    rclcpp::CallbackGroup::SharedPtr low_freq_group_;
    rclcpp::CallbackGroup::SharedPtr mutex_group_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    
    // ============================================
    // 1. 创建节点
    // ============================================
    auto node = rclcpp::Node::make_shared("bt_node_unified");

    // ============================================
    // 2. 初始化回调组管理器
    // ============================================
    CallbackGroupManager cg_manager;
    cg_manager.init(node);

    // ============================================
    // 3. 获取XML路径
    // ============================================
    std::string base_dir = ament_index_cpp::get_package_share_directory("hnurm_small_decision");
    std::string path = base_dir + "/param/tree.xml";

    // ============================================
    // 4. 注册BT节点
    // ============================================
    BT::BehaviorTreeFactory factory;
    BT::SharedLibrary loader;

    // 注册示例节点（展示统一回调组方案）
    factory.registerNodeType<hnurm_unified::ExampleUnifiedNode>("ExampleUnifiedNode");
    
    // 可以在这里注册其他节点...
    // factory.registerNodeType<...>("...");

    // ============================================
    // 5. 配置黑板
    // ============================================
    BT::Blackboard::Ptr blackboard = BT::Blackboard::create();
    blackboard->set("node", node);
    
    // 关键：将回调组管理器放入黑板，供BT节点获取
    blackboard->set("callback_group_high", cg_manager.high_freq());
    blackboard->set("callback_group_medium", cg_manager.medium_freq());
    blackboard->set("callback_group_low", cg_manager.low_freq());
    blackboard->set("callback_group_mutex", cg_manager.mutex());
    
    // 为了方便，也提供直接获取管理器的方法
    // blackboard->set("callback_group_manager", &cg_manager);
    
    // Nav2/其他配置
    blackboard->set("server_timeout", std::chrono::milliseconds(100000));
    blackboard->set("bt_loop_duration", std::chrono::milliseconds(20));  // 50Hz对应20ms
    blackboard->set("wait_for_service_timeout", std::chrono::milliseconds(100000));

    // ============================================
    // 6. 创建行为树
    // ============================================
    auto tree = factory.createTreeFromFile(path, blackboard);

    // ============================================
    // 7. 配置统一线程池执行器（关键！）
    // ============================================
    // 使用MultiThreadedExecutor作为统一线程池
    // 线程数建议：CPU核心数 × 1.5，这里假设4核CPU用6个线程
    const size_t thread_pool_size = 6;
    
    rclcpp::executors::MultiThreadedExecutor executor(
        rclcpp::ExecutorOptions(), 
        thread_pool_size,           // 线程池大小
        true,                       // yield_before_execute
        std::chrono::milliseconds(1) // timeout
    );
    
    executor.add_node(node);
    
    RCLCPP_INFO(node->get_logger(), 
        "MultiThreadedExecutor started with %zu threads", thread_pool_size);

    // ============================================
    // 8. 在独立线程中运行ROS执行器
    // ============================================
    // 这是关键：ROS回调处理与BT tick分离
    std::atomic<bool> ros_running{true};
    std::thread executor_thread([&executor, &ros_running, node]() {
        RCLCPP_INFO(node->get_logger(), "ROS executor thread started");
        
        // 使用spin()持续处理回调，或使用spin_some()定期检查停止标志
        while (ros_running.load()) {
            executor.spin_some(std::chrono::milliseconds(10));
            // 小睡眠避免忙等
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        
        RCLCPP_INFO(node->get_logger(), "ROS executor thread stopped");
    });

    // ============================================
    // 9. 主线程专用于BT tick（保证实时性）
    // ============================================
    RCLCPP_INFO(node->get_logger(), "BT tick thread started at 50Hz");
    
    rclcpp::Rate loop_rate(50);  // 50 Hz
    int tick_count = 0;
    auto start_time = std::chrono::steady_clock::now();
    
    while (rclcpp::ok()) {
        // 主线程只执行tick，不会被ROS回调阻塞
        // 因为所有ROS回调都在executor_thread中处理
        tree.tickRoot();
        
        // 简单的频率监控（每100次tick打印一次）
        tick_count++;
        if (tick_count % 100 == 0) {
            auto elapsed = std::chrono::steady_clock::now() - start_time;
            double actual_freq = 100.0 / std::chrono::duration<double>(elapsed).count();
            RCLCPP_DEBUG(node->get_logger(), 
                "Tick count: %d, actual freq: %.2f Hz", tick_count, actual_freq);
            start_time = std::chrono::steady_clock::now();
        }
        
        loop_rate.sleep();
    }

    // ============================================
    // 10. 清理
    // ============================================
    RCLCPP_INFO(node->get_logger(), "Shutting down...");
    
    ros_running = false;
    executor.cancel();
    
    if (executor_thread.joinable()) {
        executor_thread.join();
    }
    
    rclcpp::shutdown();
    return 0;
}
