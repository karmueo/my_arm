# 项目标题

## 目录

- [关于](#about)
- [入门指南](#getting_started)
- [使用方法](#usage)

## 关于 <a name = "about"></a>

使用 ROS 2 Control 和 Gazebo (经典版本也就是classic)创建和控制机械臂。
![gazebo arm](https://github.com/karmueo/my_arm/blob/master/resources/gazebo%E4%BB%BF%E7%9C%9F.png)

## 入门指南 <a name = "getting_started"></a>

以下是将项目复制到本地机器上进行开发和测试的指南。

### 先决条件

安装此软件所需的内容以及如何安装它们。
ros2 humble
gazebo classic11
ros2 control

### 安装

逐步指导如何设置开发环境。

1. 克隆项目仓库：

```bash
git clone https://github.com/karmueo/my_arm.git
```

2. 进入项目目录：

```bash
cd yourproject
```

3. 安装依赖项：

```bash
sudo apt-get update
sudo apt install ros-$ROS_DISTRO-gazebo-ros-pkgs
sudo apt-get install ros-${ROS_DISTRO}-ros2-control ros-${ROS_DISTRO}-ros2-controllers ros-${ROS_DISTRO}-gripper-controllers ros-${ROS_DISTRO}-gazebo-ros2-control
```
编译

```bash
colcon build \
        --merge-install \
        --symlink-install
```

最后给出从系统中获取数据或进行小型演示的示例。

```bash
source install/setup.bash
# 运行示例
ros2 launch my_arm robot_gazebo.launch.py
```

## 使用方法 <a name = "usage"></a>

控制机械臂运动

```bash
ros2 topic pub --once /arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{
        joint_names: [
            'shoulder_joint', 
            'upperArm_joint', 
            'foreArm_joint', 
            'wrist1_joint', 
            'wrist2_joint', 
            'wrist3_joint'
        ], 
        points: [
            {
                positions: [0.0, 0.0, 1.57, 0, 1.57, -1.57], 
                time_from_start: {sec: 5, nanosec: 0}
            }
        ]
    }"
```

