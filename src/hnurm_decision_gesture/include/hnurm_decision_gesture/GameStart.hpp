#pragma once

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <behaviortree_cpp_v3/condition_node.h>
#include <hnurm_interfaces/msg/vision_recv_data.hpp>

namespace hnurm_behavior_trees
{

    class GameStart : public BT::ConditionNode
    {
    public:
        GameStart(
            const std::string &condition_name,
            const BT::NodeConfiguration &conf);

        ~GameStart();

        GameStart() = delete;

        BT::NodeStatus tick() override;

        static BT::PortsList providedPorts()
        {
            return {
                BT::InputPort<float>("game_progress_threshold", 0.5f, "game_progress大于此值返回SUCCESS")};
        }

    private:
        void gameStartCallback(const hnurm_interfaces::msg::VisionRecvData::SharedPtr msg);

        rclcpp::Node::SharedPtr node_;

        rclcpp::Subscription<hnurm_interfaces::msg::VisionRecvData>::SharedPtr game_start_sub_;

        float game_progress_; // 当前游戏进度
        float threshold_;     // 阈值
        std::mutex mutex_;
    };

} // namespace hnurm_behavior_trees2