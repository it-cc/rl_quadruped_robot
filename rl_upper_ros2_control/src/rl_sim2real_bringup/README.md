# RL Sim2Real Bringup Package

该包提供了用于 sim2real 强化学习机器人部署的启动文件和配置。

## 功能

- 完整的 sim2real 启动配置
- 基于 YAML 的参数管理
- 硬件接口集成
- RL 控制器部署
- RViz 可视化支持

## 文件结构

```
rl_sim2real_bringup/
├── launch/
│   └── sim2real.launch.py          # 主启动文件
├── config/
│   ├── sim2real_params.yaml        # 主配置文件
│   ├── hardware_params.yaml        # 硬件专用配置
│   └── sim2real.rviz               # RViz 配置
├── CMakeLists.txt
├── package.xml
└── README.md
```

## 使用方法

### 基本启动

```bash
ros2 launch rl_sim2real_bringup sim2real.launch.py
```

### 指定 RL 模型

```bash
ros2 launch rl_sim2real_bringup sim2real.launch.py \
    model_path:=/path/to/your/model.pt
```

### 使用自定义配置

```bash
ros2 launch rl_sim2real_bringup sim2real.launch.py \
    config_file:=/path/to/custom_params.yaml
```

### 启用 RViz 可视化

```bash
ros2 launch rl_sim2real_bringup sim2real.launch.py \
    use_rviz:=true
```

### 调试模式

```bash
ros2 launch rl_sim2real_bringup sim2real.launch.py \
    log_level:=debug
```

### 完整示例

```bash
ros2 launch rl_sim2real_bringup sim2real.launch.py \
    model_path:=/home/user/models/quadruped_policy.pt \
    use_rviz:=true \
    log_level:=info
```

## 配置文件说明

### sim2real_params.yaml

主配置文件，包含：
- 机器人描述参数
- 控制器管理器参数
- 硬件接口参数
- RL 控制器参数
- 关节状态广播器参数
- Robot State Publisher 参数
- 日志参数

### hardware_params.yaml

硬件专用配置，包含：
- 串口通信参数
- 电机参数（ID、方向、偏移、限制）
- IMU 传感器参数
- 状态估计参数
- 安全监控参数

## 启动参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `config_file` | string | sim2real_params.yaml | 配置文件路径 |
| `model_path` | string | "" | RL 模型文件路径 |
| `urdf_file` | string | my_quadruped.urdf | URDF 文件路径 |
| `use_rviz` | bool | false | 是否启动 RViz2 |
| `log_level` | string | info | 日志级别 |

## 节点说明

### robot_state_publisher
发布机器人的 TF 变换和状态信息。

### controller_manager
管理和协调所有控制器的生命周期。

### joint_state_broadcaster
广播关节状态到 `/joint_states` 话题。

### rl_controller
执行强化学习策略进行机器人控制。

## 话题

### 订阅
- `/imu/data` (sensor_msgs/Imu) - IMU 数据
- `/emergency_stop` (std_msgs/Bool) - 紧急停止信号

### 发布
- `/joint_states` (sensor_msgs/JointState) - 关节状态
- `/tf` (tf2_msgs/TFMessage) - 坐标变换

## 安全注意事项

1. **首次运行前**：检查所有关节限位配置是否正确
2. **电机方向**：确认 hardware_params.yaml 中的电机方向设置
3. **紧急停止**：确保紧急停止按钮可用
4. **测试环境**：先在安全的测试环境中验证
5. **监控参数**：关注温度、电压、电流等安全参数

## 故障排除

### 无法连接到硬件
检查串口设备：
```bash
ls -l /dev/ttyUSB*
```

授予串口权限：
```bash
sudo usermod -a -G dialout $USER
```

### 控制器启动失败
查看日志：
```bash
ros2 topic echo /controller_manager/diagnostics
```

### IMU 数据异常
检查 IMU 话题：
```bash
ros2 topic echo /imu/data
```

## 依赖

- robot_state_publisher
- controller_manager
- joint_state_broadcaster
- rl_sim2real_controller
- rl_sim2real_hardware_interface
- rl_sim2real_robot_description
- rviz2 (可选)

## 开发与调试

### 查看活动控制器
```bash
ros2 control list_controllers
```

### 查看硬件接口
```bash
ros2 control list_hardware_interfaces
```

### 监控关节状态
```bash
ros2 topic echo /joint_states
```

### 检查 TF 树
```bash
ros2 run tf2_tools view_frames
```

## 许可证

TODO: License declaration

## 维护者

cc (3300833815@qq.com)
