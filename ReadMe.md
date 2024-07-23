# AGV电池信息采集器

该脚本从自动导引车（AGV）收集电池信息并将其发送到指定的服务器端点。它订阅ROS消息并将数据发布到RESTful API。

## 功能

- 使用ROS消息收集AGV的电池信息。
- 将收集到的数据发布到指定的RESTful API端点。
- 通过命令行参数进行配置。

## 要求

- Python 3.x
- ROS（机器人操作系统）
- `rospy` 包
- 包含 `ModuleInformation` 和 `BatteryState` 消息的 `gpm_msgs` 包

## 安装

1. 确保已安装Python 3和ROS。
2. 安装必要的ROS包：
    ```bash
    sudo apt-get install ros-<your-ros-distro>-rospy
    sudo apt-get install ros-<your-ros-distro>-gpm-msgs
    ```
3. 克隆此仓库并导航到项目目录：
    ```bash
    git clone <repository_url>
    cd <repository_directory>
    ```

## 使用

使用所需参数运行脚本：

```bash
./battery_info_collector.py --fieldName <字段名称> --agvName <AGV名称> --host <主机URL>
