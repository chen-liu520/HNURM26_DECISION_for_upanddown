#include "hnurm_decision_gesture/GameStart.hpp"
#include <termios.h>
#include <unistd.h>
#include <iostream>

namespace hnurm_behavior_trees
{

    GameStart::GameStart(const std::string &condition_name, const BT::NodeConfiguration &conf)
        : BT::ConditionNode(condition_name, conf),
          game_progress_(0.0f), // 默认初始为0，避免意外
          threshold_(0.5f)
    {
        node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

        // 从端口读取阈值参数
        if (!getInput<float>("game_progress_threshold", threshold_))
        {
            threshold_ = 1.0f;
        }

        RCLCPP_INFO(node_->get_logger(), "[GameStart] 初始化完成，等待game_progress > %.1f...", threshold_);

        // 创建订阅，监听 /vision_recv_data
        // 使用默认 QoS（与 ros2 topic echo 一致）
        game_start_sub_ = node_->create_subscription<hnurm_interfaces::msg::VisionRecvData>(
            "/vision_recv_data",
            rclcpp::QoS(10),
            std::bind(&GameStart::gameStartCallback, this, std::placeholders::_1));
    }

    GameStart::~GameStart()
    {
    }

    BT::NodeStatus GameStart::tick()
    {
        std::lock_guard<std::mutex> lock(mutex_);

        /*
        if (game_progress_ > threshold_)
        {
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::FAILURE;*/
        return BT::NodeStatus::SUCCESS;
    }

    void GameStart::gameStartCallback(const hnurm_interfaces::msg::VisionRecvData::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        game_progress_ = msg->game_progress;
    }
}

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<hnurm_behavior_trees::GameStart>("GameStart");
}