#include "Status.hpp"

namespace hnurm_behavior_trees
{
    Status::Status(const std::string &condition_name, const BT::NodeConfiguration &conf) : BT::ConditionNode(condition_name, conf),
                                                                                           status_(false)
    {

    } // 内部状态status_初始化为false

    BT::NodeStatus Status::tick()
    {
        bool status = false;
        getInput("Status", status); // 通过getInput()方法从端口"Status"获取黑板上的状态值，存储在局部变量status中

        if (status)
        {
            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            return BT::NodeStatus::FAILURE;
        }
    }
}
