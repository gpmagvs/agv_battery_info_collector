# 使用方式
## 安裝
1. 將專案資料夾放置到 ROS 工作空間的 src 目錄中：
   如果您已經下載了agv_battery_info_collector資料夾，直接將其複製到 `~/catkin_ws/src` 目錄中：

2. 返回 ROS 工作空間的根目錄，並執行 catkin_make 來編譯套件：

   ```bash
   cd ~/catkin_ws
   catkin_make
   ```

3. 修改啟動檔案參數

   打開 `launch/agv_battery_info_collector.launch` 檔案，根據需要修改以下參數：

   - `host`: 伺服器地址和端口，預設為 "192.168.0.3:5254"
   - `agvName`: AGV 名稱，預設為 "AGV_001"
   - `fieldName`: 區域名稱，預設為 "Unknown_Region"
   - `pubInterval`: 資料發布間隔（秒），預設為 10

   例如：

   ```xml
   <arg name="host" default="192.168.1.100:5000"/>
   <arg name="agvName" default="AGV_002"/>
   <arg name="fieldName" default="Warehouse_A"/>
   <arg name="pubInterval" default="5"/>
   ```

4. 啟動節點

   使用以下指令啟動 AGV 電池資訊收集器節點：

   ```
   roslaunch agv_battery_info_collector agv_battery_info_collector.launch
   ```

   如果需要覆蓋預設參數，可以在啟動指令中指定：

   ```
   roslaunch agv_battery_info_collector agv_battery_info_collector.launch host:=192.168.1.100:5000 agvName:=AGV_002 fieldName:=Warehouse_A pubInterval:=5
   ```

啟動後，節點將訂閱 `/module_information` 主題，收集電池資訊並按照指定的間隔將資料發送到伺服器。




5. 將啟動指令添加到 Shell 腳本

   如果您使用 Shell 腳本來啟動多個程序(例如車載電腦中的agvc.sh)，請將 AGV 電池資訊收集器的啟動指令添加到您的腳本中。例如，您可以在腳本中添加以下行：

   ```bash
   roslaunch agv_battery_info_collector agv_battery_info_collector.launch
   ```

   如果需要使用自定義參數，可以這樣寫：

   ```bash
   roslaunch agv_battery_info_collector agv_battery_info_collector.launch host:=192.168.1.100:5000 agvName:=AGV_002 fieldName:=Warehouse_A pubInterval:=5
   ```

   確保將此行添加到適當的位置，以便與其他程序一起啟動。

