# 指令

## hnurm_decision_gesture 包
`ros2 launch hnurm_decision_gesture hnurm_decision.launch.py`

## run运行
`ros2 run hnurm_decision_gesture decision_node --ros-args --params-file ~/DECISION/special_upanddown_decision/src/hnurm_decision_gesture/param/params.yaml`

## gimbal_rotation_server 包

### 运行云台旋转服务节点
`ros2 run gimbal_rotation_server gimbal_rotation_server`

### 运行模拟裁判系统节点
`ros2 run gimbal_rotation_server referee`

### 运行姿态打印节点
`ros2 run gimbal_rotation_server gesture_printer`

### 运行TF测试节点
`ros2 run gimbal_rotation_server test_tf`