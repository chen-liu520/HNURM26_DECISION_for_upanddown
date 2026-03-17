# 统一线程池方案 vs 旧方案对比

## 架构对比图

### 旧方案（每个节点独立线程）
```
┌─────────────────────────────────────────────────────────────┐
│                     Main Thread                             │
│  ┌─────────────────┐      ┌──────────────────────────┐      │
│  │ MultiThreaded   │      │   tree.tickRoot() @50Hz  │      │
│  │ Executor        │      │                          │      │
│  │ (Nav2回调)      │──────→│   tick()                 │      │
│  └─────────────────┘      │   - 读取数据              │      │
│                           └──────────────────────────┘      │
└─────────────────────────────────────────────────────────────┘
        │                           │
        ▼                           ▼
┌───────────────┐           ┌───────────────┐
│ PubRobotStatus│           │  OtherNode1   │
│ 独立线程       │           │  独立线程      │
│ ├─Executor    │           │ ├─Executor    │
│ ├─Thread1     │           │ ├─Thread1     │
│ ├─Thread2     │           │ └─Thread2     │
│ └─...         │           └───────────────┘
└───────────────┘                     │
                                      ▼
                              ┌───────────────┐
                              │  OtherNode2   │
                              │  独立线程      │
                              │ └─Thread      │
                              └───────────────┘

问题：
- 10个BT节点 = 10+个线程
- 线程管理复杂
- 上下文切换开销
- 难以统一调优
```

### 新方案（统一线程池）
```
┌─────────────────────────────────────────────────────────────┐
│                   Unified Thread Pool                       │
│              (MultiThreadedExecutor, 6 threads)             │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│   ┌──────────────┐  ┌──────────────┐  ┌──────────────┐     │
│   │ HIGH FREQ    │  │ MEDIUM FREQ  │  │ LOW FREQ     │     │
│   │ CallbackGroup│  │ CallbackGroup│  │ CallbackGroup│     │
│   │ (Reentrant)  │  │ (Reentrant)  │  │ (Reentrant)  │     │
│   │              │  │              │  │              │     │
│   │ • PubRobot   │  │ • PubRobot   │  │ • PubRobot   │     │
│   │   /sensor    │  │   /command   │  │   /config    │     │
│   │ • OtherNode  │  │ • OtherNode  │  │ • OtherNode  │     │
│   │   /odom      │  │   /target    │  │   /referee   │     │
│   └──────────────┘  └──────────────┘  └──────────────┘     │
│                                                             │
│   所有订阅者共享6个线程，按频率分组管理                         │
│                                                             │
└─────────────────────────────────────────────────────────────┘
                              │
                    ┌─────────┴─────────┐
                    ▼                   ▼
            ┌───────────────┐    ┌───────────────┐
            │ Executor      │    │ Main Thread   │
            │ Thread        │    │               │
            │ (ROS callbacks)│   │ tree.tickRoot()│
            └───────────────┘    │ @ 50Hz        │
                                 │               │
                                 │ - 读取原子变量 │
                                 │ - 无阻塞      │
                                 └───────────────┘

优势：
- 10个BT节点 = 6个线程（固定）
- 集中管理，易于调优
- 减少上下文切换
- tick与ROS回调完全隔离
```

---

## 代码对比

### 旧方案（PubRobotStatus）
```cpp
class PubRobotStatus : public BT::SyncActionNode {
private:
    // ===== 每个节点都创建这些 =====
    rclcpp::CallbackGroup::SharedPtr callback_group_;
    rclcpp::CallbackGroup::SharedPtr callback_group_for_vision_data_;
    
    rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
    rclcpp::executors::SingleThreadedExecutor callback_group_executor_for_vision_data_;
    
    std::thread executor_thread_;  // 独立线程！
    
public:
    PubRobotStatus(...) : BT::SyncActionNode(...) {
        node_ = conf.blackboard->get<rclcpp::Node::SharedPtr>("node");
        
        // 创建回调组
        callback_group_ = node_->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive, false);
        
        // 创建执行器
        callback_group_executor_.add_callback_group(
            callback_group_, node_->get_node_base_interface());
        
        // 创建独立线程运行执行器！
        executor_thread_ = std::thread([this]() {
            callback_group_executor_.spin();
        });
        
        // 创建订阅者
        rclcpp::SubscriptionOptions sub_option;
        sub_option.callback_group = callback_group_;
        
        sub_ = node_->create_subscription<...>(..., sub_option);
    }
    
    ~PubRobotStatus() {
        // 必须join线程
        if (executor_thread_.joinable()) {
            executor_thread_.join();
        }
    }
};
```

### 新方案（ExampleUnifiedNode）
```cpp
class ExampleUnifiedNode : public BT::SyncActionNode {
private:
    // ===== 不再创建这些！=====
    // rclcpp::CallbackGroup::SharedPtr callback_group_;  // 从黑板获取
    // rclcpp::executors::SingleThreadedExecutor executor_;  // 不需要
    // std::thread executor_thread_;  // 不需要！
    
    // 只保留数据成员
    std::atomic<double> sensor_value_;
    std::mutex data_mutex_;
    
public:
    ExampleUnifiedNode(...) : BT::SyncActionNode(...) {
        node_ = conf.blackboard->get<rclcpp::Node::SharedPtr>("node");
        
        // ===== 关键：从黑板获取共享回调组 =====
        auto high_freq_cg = config().blackboard->get<
            rclcpp::CallbackGroup::SharedPtr>("callback_group_high");
        auto medium_freq_cg = config().blackboard->get<
            rclcpp::CallbackGroup::SharedPtr>("callback_group_medium");
        
        // ===== 创建订阅者，使用共享回调组 =====
        rclcpp::SubscriptionOptions high_option;
        high_option.callback_group = high_freq_cg;  // 使用共享组
        
        sensor_sub_ = node_->create_subscription<...>(
            "/sensor", ..., high_option);  // 指定回调组
        
        // ===== 不再创建执行器和线程！=====
        // 所有回调由decision2.cpp中的统一线程池调度
    }
    
    ~ExampleUnifiedNode() {
        // ===== 不需要join线程！=====
        // 统一线程池在decision2.cpp中管理
    }
    
    BT::NodeStatus tick() override {
        // 直接读取数据，无需spin_some()
        double value = sensor_value_.load();
        // ... 处理逻辑
        return BT::NodeStatus::SUCCESS;
    }
};
```

---

## decision.cpp vs decision2.cpp

### decision.cpp（旧）
```cpp
int main(...) {
    // ...
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    
    rclcpp::Rate loop_rate(50);
    while (rclcpp::ok()) {
        executor.spin_some();  // 可能阻塞tick！
        tree.tickRoot();       // 被ROS回调延迟
        loop_rate.sleep();
    }
}
```

### decision2.cpp（新）
```cpp
int main(...) {
    // ...
    // 1. 创建回调组管理器
    CallbackGroupManager cg_manager;
    cg_manager.init(node);
    
    // 2. 将回调组放入黑板
    blackboard->set("callback_group_high", cg_manager.high_freq());
    // ...
    
    // 3. 统一线程池
    rclcpp::executors::MultiThreadedExecutor executor(
        rclcpp::ExecutorOptions(), 6);  // 6个线程
    executor.add_node(node);
    
    // 4. ROS回调在独立线程
    std::thread executor_thread([&executor]() {
        while (running) {
            executor.spin_some(std::chrono::milliseconds(10));
        }
    });
    
    // 5. 主线程专用于tick
    rclcpp::Rate loop_rate(50);
    while (rclcpp::ok()) {
        tree.tickRoot();  // 不受ROS回调影响！
        loop_rate.sleep();
    }
    
    executor_thread.join();
}
```

---

## 关键区别总结

| 方面 | 旧方案 | 新方案 |
|-----|-------|-------|
| **线程数量** | 随节点增加（N节点×2线程） | 固定（如6个线程） |
| **线程管理** | 分散在各BT节点 | 集中在decision2.cpp |
| **tick隔离性** | 可能被ROS回调阻塞 | 完全隔离，保证50Hz |
| **资源占用** | 高 | 低且可控 |
| **调优难度** | 难（需修改各节点） | 易（只改decision2.cpp） |
| **代码复杂度** | 高（每节点管理线程） | 低（只创建订阅者） |

---

## 迁移指南

将旧BT节点迁移到新方案的步骤：

1. **删除成员变量**
   ```cpp
   // 删除这些
   rclcpp::CallbackGroup::SharedPtr callback_group_;
   rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
   std::thread executor_thread_;
   ```

2. **修改构造函数**
   ```cpp
   // 旧
   callback_group_ = node_->create_callback_group(...);
   callback_group_executor_.add_callback_group(...);
   executor_thread_ = std::thread([...] { executor.spin(); });
   
   // 新
   auto cg = config().blackboard->get<rclcpp::CallbackGroup::SharedPtr>(
       "callback_group_medium");
   ```

3. **修改订阅者创建**
   ```cpp
   // 旧
   sub_ = node_->create_subscription<...>(..., callback_group_);
   
   // 新
   rclcpp::SubscriptionOptions opt;
   opt.callback_group = cg;
   sub_ = node_->create_subscription<...>(..., opt);
   ```

4. **删除析构函数中的join**
   ```cpp
   // 删除
   if (executor_thread_.joinable()) executor_thread_.join();
   ```

5. **tick()函数不变**，但不再需要`executor.spin_some()`
