
#include <rclcpp/rclcpp.hpp>

#include <string>
#include <behaviortree_cpp_v3/condition_node.h>





namespace hnurm_behavior_trees {

    class Status : public BT::ConditionNode {
    public:
        /**
         * @brief A constructor for nav2_behavior_tree::Status
         * @param condition_name Name for the XML tag for this node
         * @param conf BT node configuration
         */
        Status(const std::string & condition_name, const BT::NodeConfiguration & conf);

        Status() = delete;

        /**
         * @brief The main override required by a BT action
         * @return BT::NodeStatus Status of tick execution
         */
        
        BT::NodeStatus tick() override;

        /**
         * @brief Creates list of BT ports
         * @return BT::PortsList Containing node-specific ports
         */
        static BT::PortsList providedPorts()
        {
            return
            {
                // 定义了一个输入端口"Status"，类型为bool
                // 端口的描述：用于判断是否执行后续行为的状态
                BT::InputPort<bool>("Status", "status to judge whether excute ")
            };
        }


    private:
        bool status_ ;

    };

} // namespace hnurm_behavior_trees