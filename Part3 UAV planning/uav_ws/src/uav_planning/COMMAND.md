# PX4 起飞悬停启动命令

## 1. 基本用法

以下命令在 WSL Ubuntu 终端执行。

```bash
# 每个新终端先加载工作区环境，使 ROS 2 能找到本项目的包。
source /home/ruan/uav_ws/install/setup.bash

# 使用默认参数启动：打开 Gazebo 界面，自动解锁、起飞并悬停。
# 当前 YAML 默认目标为相对起飞点上升 5 米。
ros2 launch uav_planning takeoff_hold.launch.py
```

命令各部分的含义：

```text
ros2 launch                 启动 ROS 2 launch 文件
uav_planning                ROS 2 包名
takeoff_hold.launch.py      启动文件名
参数名:=值                  覆盖某个启动参数；可以在命令后添加多个
```

不写的参数自动使用默认值，不需要把所有参数都列出来。

## 2. 启动参数

下表中的“包共享目录”由 ROS 2 查找；当前工作区默认是：

```text
/home/ruan/uav_ws/install/uav_planning/share/uav_planning
```

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `gui` | `true` | 是否打开 Gazebo 图形界面。设为 `false` 时，仿真服务端仍然运行。只接受小写 `true`、`false`。 |
| `start_controller` | `true` | 是否启动 `takeoff_hold_node`。当前默认配置会自动请求 Offboard、解锁并起飞；设为 `false` 则不启动控制节点。只接受小写 `true`、`false`。 |
| `target_altitude` | 空字符串 | 相对初始起飞位置的目标高度，单位米，允许范围 `[0.5, 30]`。未指定时，使用参数 YAML 中的值，当前为 `5.0`。不是地图绝对高度，也不是在当前高度上再增加该数值。 |
| `px4_dir` | `/home/ruan/PX4-Autopilot` | 已编译的 PX4 仓库目录。默认按当前用户主目录生成。 |
| `runtime_dir` | `/home/ruan/uav_ws/run/px4_sitl` | PX4 运行目录，保存仿真参数、日志等；不存在时自动创建。默认按当前用户主目录生成。 |
| `dds_port` | `8888` | DDS Agent 的 UDP 端口，PX4 客户端同步使用此端口。允许范围 `[1024, 65535]`，且不能被占用。 |
| `world` | 包共享目录下的 `worlds/flood_demo.sdf` | Gazebo 地图文件。当前启动文件要求地图包含名为 `x500_0` 的官方 X500 `<include>`。 |
| `params_file` | 包共享目录下的 `config/takeoff_hold.yaml` | 控制节点的 ROS 2 参数 YAML，包含默认高度、控制频率、反馈超时和自动解锁开关等。 |
| `bridge_config` | 包共享目录下的 `config/bridge.yaml` | Gazebo 与 ROS 2 的桥接配置。当前桥接 `/clock` 和用于验证的真值里程计，不通过此桥接发送 PX4 飞行控制指令。 |

初期主要使用 `gui`、`start_controller`、`target_altitude`，其余参数通常保持默认即可。

## 3. 常用命令与注释

```bash
# 打开界面，自动起飞到相对起飞点 8 米并保持。
ros2 launch uav_planning takeoff_hold.launch.py target_altitude:=8.0

# 不打开界面，自动起飞到相对起飞点 2 米；适合无界面测试。
ros2 launch uav_planning takeoff_hold.launch.py gui:=false target_altitude:=2.0

# 只启动仿真和通信链路，不启动自动起飞控制节点。
ros2 launch uav_planning takeoff_hold.launch.py start_controller:=false

# 同时明确指定三个常用参数。
# 行尾的反斜杠表示下一行仍属于同一条命令；反斜杠后不要加注释或空格。
ros2 launch uav_planning takeoff_hold.launch.py \
  gui:=true \
  start_controller:=true \
  target_altitude:=5.0

# 查看启动参数及其默认值；只显示参数，不启动仿真。
ros2 launch uav_planning takeoff_hold.launch.py --show-args
```

## 4. 使用时注意

- 默认启动 Gazebo 服务端、Micro XRCE-DDS Agent、PX4 SITL、Gazebo–ROS 2 桥接节点和起飞控制节点；`gui` 单独控制图形界面是否启动。
- `start_controller:=false` 仅表示不启动本项目的控制节点；如果后续接入其他控制程序，无人机仍可能接收它的指令。
- `target_altitude` 是启动时的配置，不是飞行中发布的指令。当前节点将其设为只读，并且只在初始化时计算一次位置目标，因此不能用 `ros2 param set` 在飞行中改高度。
- `/target_altitude` 订阅接口尚未实现，目前不能通过向该话题发布消息来改高度。也不要另起发布器向 `/fmu/in/trajectory_setpoint` 抢发目标，以免与当前控制节点冲突。
- 同一时间只启动一套本项目仿真。不要在已有 PX4 或 Agent 运行时重复执行启动命令。
- `Ctrl+C` 会结束整套 launch 和仿真，不等于让无人机完成物理降落。

## 5. T0：只记录地面坐标参考，不起飞

终端 1：使用已有启动文件，但不启动控制节点。不要同时运行其他飞行控制程序。

```bash
source /home/ruan/uav_ws/install/setup.bash
ros2 launch uav_planning takeoff_hold.launch.py start_controller:=false
```

终端 2：在同一 ROS domain 的环境中运行只读记录脚本。

```bash
source /home/ruan/uav_ws/install/setup.bash
python3 /home/ruan/uav_ws/src/uav_planning/test/validate_goto_point.py
```

脚本等待有效 PX4 定位、未解锁状态与地面静止条件，然后采样2仿真秒并退出。
它只订阅消息，不发送解锁/位置目标，也不自动修改配置。当前尚未实现单点飞行验证模式。

当前已核验的机体地图参考点为 `[-24.0,-18.0,0.227]` 米，保存在
`/home/ruan/uav_ws/src/uav_planning/config/goto_point.yaml`。
这个 Z 是 base_link 原点的地图高度，不是出生高度0.3米，也不是脚架接地点高度0。

原始真值话题实际记录模型原点，模型到 base_link 有0.24米偏移，记录脚本已按姿态补偿。
PX4 的 NED 参考值每次启动重新获取，不把脚本打印的某次 NED 数值存成固定配置。
更换模型、起飞位置或地面后，应重新采样并核对地图参考值；后续控制节点仍只能使用 PX4 估计闭环，不能把真值接入连续控制。

## 6. T1：构建和验证独立坐标转换模块

本模块是纯 C++ 工具库，不是 ROS 2 节点，不需要启动 Gazebo 或 PX4，也不能通过 `ros2 run` 单独运行。

```bash
source /home/ruan/uav_ws/install/setup.bash
cd /home/ruan/uav_ws

# 构建现有包、新的坐标转换静态库和单元测试。
colcon build --packages-select uav_planning --symlink-install

# 只运行9项坐标数学测试；不会解锁或启动仿真。
colcon test --packages-select uav_planning \
  --ctest-args -R '^test_frame_alignment$' --output-on-failure

# 查看包内已有测试结果；其中也可能包含此前运行过的检查。
colcon test-result --test-result-base /home/ruan/uav_ws/build/uav_planning --verbose
```

接口阅读顺序：

1. `/home/ruan/uav_ws/src/uav_planning/include/uav_planning/frame_alignment.hpp`：参考点数据与四个公开方法。
2. `/home/ruan/uav_ws/src/uav_planning/src/frame_alignment.cpp`：初始化及正反坐标转换。
3. `/home/ruan/uav_ws/src/uav_planning/test/test_frame_alignment.cpp`：非零参考点、正负轴向、往返转换和错误输入的示例。

使用时先调用 `initialize(map_reference, ned_reference)`，再调用 `map_to_ned()` 或 `ned_to_map()`。
地图参考点来自 T0 配置，NED 对应参考点须由后续节点在本次启动时取得。
本模块只转换位置，不转换姿态，也不额外叠加模型原点到 base_link 的0.24米偏移。

## 7. 当前地图：P1–P6楼顶配送点

地图有6栋底面6×6米的配送楼，保留原有B1–B5；最高楼仍是B3的14米。P1、P2的楼体与楼顶标记一起平移，其余位置保持不变。以绿色基地中心(-23,-17,0)为参考，到A/B/C（P1/P2/P3）楼顶标记的三维直线距离约为22.76、37.94、53.11米，比例为1.5:2.5:3.5。机号和任务顺序仍由原分配JSON提供。

| 目标 | X | Y | 楼顶高度Z（米） | 楼顶上方2米访问高度 |
| --- | ---: | ---: | ---: | ---: |
| P1 | -23 | 5.207 | 5 | 7 |
| P2 | -6 | 15.959 | 8 | 10 |
| P3 | 16 | 17 | 12 | 14 |
| P4 | 24 | 4 | 5 | 7 |
| P5 | 21 | -16 | 8 | 10 |
| P6 | 4 | -19 | 12 | 14 |

黄色小球半径由0.225米减为0.10米，球心位于标记原点上方0.25米；橙色圆盘与黄色球都只有visual，真正参与碰撞的是楼体。

**原来的P1低空直飞和所有Z=2的旧P2/P3/P4路线已不适用于新地图。** 路径文件已整体更新；不要从旧聊天记录复制低空航点。楼顶标记的Z表示屋顶表面，不是机体悬停或接触时的base_link高度。

map参考为[-24,-18,0.227]米，NED参考每次启动重新锁定。planner_node读取当前SDF的11栋建筑collision，在三维栅格中执行A*；goto_point_node负责起飞、轨迹跟踪和末端降落。地图仍是已知静态场景，没有实时感知或动态障碍重规划。

## 8. 当前航线：访问P2 → P3 → P4，具体路径由三维A*决定

默认示例使用下表三个map绝对坐标访问点。实际任务由发布节点提供，可更换点数、顺序和坐标；A*不把P2/P3/P4或旧16米中转点写死在算法中：

| 顺序 | 任务访问点 | 坐标（米） |
| --- | --- | --- |
| 1 | P2楼顶上方2米 | (-6,15.959,10) |
| 2 | P3楼顶上方2米 | (16,17,14) |
| 3 | P4楼顶上方2米 | (24,4,7) |

执行节点先在原出生位置相对起飞2米。A*从这个预计悬停位置开始，分三段搜索到P2、P3、P4，自动选择绕行、爬升和下降路径，再做段内无碰撞连线简化。只保留任务顺序，不要求先爬到固定高度或先飞到目标正上方。

任务访问点与执行航点不同：3个任务点经过A*后可能生成更多转弯/绕障执行点。P2、P3、P4必须依次保留；不能直接把这3点发给/uav/path，因为起点直连P2、P2直连P3、P3直连P4未必满足建筑净空。

规划采用0.5米三维栅格、26邻域和欧氏距离代价，建筑外扩1.5米并额外保留半格离散余量。每条搜索边及简化连线都检查膨胀长方体相交，避免斜穿建筑和边角。任一段无路就不发布任何执行路径。

最大参考速度1.0米/秒，加速度0.5米/秒²；每个执行点减速并停稳0.5秒后继续，不做不停顿的圆滑转弯。A*可以产生斜飞段，受楼体约束时也可能保留水平或垂直段，不强制每段都斜飞。只在最后的P4请求自主降落。

末端下降到屋顶由PX4的AUTO_LAND和落地检测负责，不把屋顶表面Z=5或预期触地机体Z=5.227作为A*飞行目标。

### 第0步：构建与重启地图

~~~bash
source /home/ruan/uav_ws/install/setup.bash
cd /home/ruan/uav_ws
colcon build --packages-select uav_planning --symlink-install
~~~

SDF和只读YAML修改后，要关闭原仿真与节点再启动新场景；已运行的Gazebo不会自动载入新地图。

### 终端1：仿真、PX4和通信

~~~bash
source /home/ruan/uav_ws/install/setup.bash
ros2 launch uav_planning takeoff_hold.launch.py start_controller:=false
~~~

不要再另外启动DDS Agent、PX4、桥接器或旧takeoff_hold_node。无界面验证可加gui:=false。

### 终端2：航点执行节点

~~~bash
source /home/ruan/uav_ws/install/setup.bash
ros2 run uav_planning goto_point_node --ros-args \
  --params-file /home/ruan/uav_ws/src/uav_planning/config/goto_point.yaml
~~~

等待日志“锁定NED参考”。当前没有任务时只发布/uav/estimated_pose，不发送飞行目标、心跳或解锁请求。goto_point.launch.py仍未实现，继续使用上述手动启动方式。

### 终端3：启动三维A*规划节点

~~~bash
source /home/ruan/uav_ws/install/setup.bash
ros2 run uav_planning planner_node --ros-args \
  --params-file /home/ruan/uav_ws/src/uav_planning/config/goto_point.yaml
~~~

该节点默认读取包内worlds/flood_demo.sdf，等待/uav/estimated_pose和/uav/mission；仅启动节点不会起飞。两个节点共用同一个YAML，起飞高度、仿真时间和区域边界在/**下只配置一次。

### 终端4：使用路线发布节点下发任务

~~~bash
source /home/ruan/uav_ws/install/setup.bash
ros2 run uav_planning mission_publisher_node --ros-args \
  -p 'waypoints:=[-6.0,15.959,10.0,16.0,17.0,14.0,24.0,4.0,7.0]'
~~~

waypoints按[x1,y1,z1,x2,y2,z2,...]排列，每3个数是一个map/ENU绝对坐标访问点，单位米；参数中的数值统一写成小数。可提供一个或多个点，顺序就是访问顺序；没有默认路线，必须由调用者提供。

该节点等待/uav/mission的订阅者出现后只发布一次，不循环下发任务；看到发送日志后可按Ctrl+C退出发布节点。规划与飞行由后续节点继续执行，不依赖发布节点一直运行。规划成功后才会触发自动起飞。

如果要保留原来的YAML消息示例，也可选择以下发布方式；两种方式一轮任务只选一种：

~~~bash
ros2 topic pub --once /uav/mission nav_msgs/msg/Path \
  "$(cat /home/ruan/uav_ws/src/uav_planning/config/p2_p3_p4_path.yaml)"
~~~

p2_p3_p4_path.yaml只是消息示例。上述旧命令由ros2 topic pub把YAML转换成Path消息；planner_node本身不读取这个路线文件。执行时保持任务开始的航向，忽略航点orientation。

~~~text
你的发布节点 → /uav/mission任务点 + 当前SDF + 起飞悬停位置
                  ↓ planner_node：三维A*与路径简化
              /uav/path
                  ↓ goto_point_node：分段加减速与速度前馈
           ENU→NED → PX4 Offboard → P4屋顶降落
~~~

主header必须为map，子header省略时继承map。先检查整条任务的关键点，再逐段搜索；整条任务成功后才发布/uav/path。当前流程只向/uav/mission发任务；直接发/uav/path或兼容接口/uav/goal会绕过A*，不要与本流程混用。一轮只接收一个任务，不支持中途替换。

### 主要参数

| 参数 | 当前值 | 含义 |
| --- | --- | --- |
| map_reference_position | [-24,-18,0.227] | 地面机体参考，NED原点每次重新获取 |
| takeoff_altitude | 2.0 | 初始相对起飞量；后续高度由A*规划 |
| max_speed / max_acceleration | 1.0 / 0.5 | 最大参考速度翻倍，加速度保留原值 |
| flight_area_min / flight_area_max | [-26,-20,1] / [26,20,18] | map外边界，内部仍有建筑 |
| auto_land | true | 只有最后执行点P4完成后请求楼顶降落 |
| control_rate / feedback_timeout | 20.0 / 1.0 | 控制参考Hz、反馈超时仿真秒 |
| resolution / clearance | 0.5 / 1.5 | A*栅格间距m、建筑净空m，另加半格离散余量 |

1m/s下仅发送位置目标会产生跟随滞后；现将分段加减速曲线的解析速度一并作为PX4位置控制的速度前馈。悬停/握手时前馈为0，速度ENU→NED只换轴和Z符号，不增加位置参考偏移。保留0.75米跟踪误差保护和到点阈值，航点停稳0.5秒后继续或降落。

### 结束与提前接管

正常完成时等待“自主降落完成：PX4已确认落地并解除锁定”，再关闭节点和仿真。P4屋顶Z=5米，机体base_link触地时预期地图Z约5.227米，不能仍以接近地面Z=0判断落地。

提前降落可执行：

~~~bash
/home/ruan/PX4-Autopilot/build/px4_sitl_default/bin/px4-commander land
~~~

节点交还飞控，不抢回Offboard或重新解锁。关闭整个仿真只会停止物理演算，不能代替完成降落。

下一轮仍须重启整套Gazebo/PX4，让飞机回到固定起飞点；不能在P4屋顶只重启控制节点并继续沿用原地面参考。

历史基线：2026-11-10以0.5m/s人工路径完成PX4/Gazebo无界面实飞。P2/P3/P4访问点水平误差约0.071/0.109/0.142米，垂直误差约0.099/0.067/0.114米；落地水平误差0.133米，机体base_link地图Z=5.227米。下面的旧日志只对应当时版本，新的A*与1m/s验证另行记录。

已解锁Offboard飞行阶段，机体原点距建筑碰撞盒的最小距离约2.063米；末端屋顶接触另行验收。首轮验收脚本曾把落地后未解锁的Offboard状态算入空中净空，现已修正统计条件，并用本次保存的飞行日志重新核验通过；没有修改控制逻辑或放宽净空阈值。

SDF格式校验和构建通过；没有GUI可视化检查。实飞和复核结果位于/tmp/uav_rooftops_20261110_pzr14heo/result_verified.json，旧地图的飞行记录保留在task.md历史段落。

### 历史基线：固定9关键点的三维A*与1m/s实跑

已实际执行完整的/uav/mission → 三维A* → /uav/path → PX4链路，9个输出航点与原路线逐项一致。读取11个碰撞盒、297675格，9段搜索共扩展17473格，耗时约226毫秒；当前各段无障碍，段内简化后输出仍为原9点。

参考速度峰值确认为1.000m/s；真值瞬时速度峰值约1.185m/s，说明max_speed是名义参考上限，并非机体绝对限速。实飞约178.67仿真秒，包含起飞、各点停稳及降落，所以任务时间没有严格减半。

P2/P3/P4访问点真值水平误差约0.053/0.109/0.157米，垂直误差约0.043/0.054/0.055米；P4屋顶落地水平误差约0.087米，base_link地图Z=5.227米。已确认落地并解除锁定；末点仅发一次降落请求，落地后3仿真秒没有后续飞行目标、心跳或重新解锁。已解锁Offboard段机体原点距建筑碰撞盒的最小距离约2.038米。

首次只改速度时触发了原0.75米跟踪保护，补上轨迹速度前馈后完成实飞。验收中位置差分速度峰值受分别采样的时间戳抖动影响，现已改为核对PX4收到的解析速度，并用同一实飞ULog的独立真值复核。没有修改保护阈值、加速度、2秒停稳或地图。

构建、9项坐标测试、6项A*测试和现有检查通过：汇总48项、0错误、0失败、9跳过，cppcheck 2.7按ament规则跳过。未检查GUI。可保留查看的结果文件为/home/ruan/uav_ws/run/astar_validation_result.json；本次完整日志位于/tmp/uav_astar_20261111_vb_2a1bs，临时目录可能在系统重启后清理。

代码阅读顺序：include/uav_planning/astar_3d.hpp → src/astar_3d.cpp → src/planner_node.cpp → src/goto_point_node.cpp中的速度前馈。

### 2026-11-11更新：只固定三个任务点

当前输入只含P2/P3/P4三个任务点。以上固定9点、226毫秒和178.67秒的记录属于上一版；下面是当时地图上A*生成和PX4实跑的历史结果，P2坐标以本页第7节为准。

| 本次执行航点 | map坐标（米） | 作用 |
| --- | --- | --- |
| 1 | (-18.5,-8.5,8) | 绕开B1边界并爬升 |
| 2 | (-7,16,10) | P2前的自动中间点 |
| 3 | (-6,18,10) | 访问P2 |
| 4 | (12,17,14) | P3前的自动爬升点 |
| 5 | (16,17,14) | 访问P3 |
| 6 | (16,12,14) | 离开P3楼体边界后再下降 |
| 7 | (24,4,7) | 访问P4，随后屋顶降落 |

这是本次A*输出的示例，不是需要填回任务文件的必经点；只向/uav/mission发送P2/P3/P4三个点。A*输出由地图、规划参数和起飞参考决定。此次规划最高14米，包含4段水平与高度同时变化的斜飞段；路径长82.55米，旧固定9点路线为116.31米。

实际按顺序完成全部7个执行点，任务约137.43仿真秒，旧固定9点且同为1m/s的实跑约178.67秒。参考速度峰值1.000m/s，已解锁Offboard阶段真值瞬态峰值约1.193m/s；另用ULog独立真值确认2290个采样同时存在水平和垂直运动。

P2/P3/P4到点真值水平误差约0.075/0.226/0.177米，垂直误差约0.065/0.016/0.055米；P4屋顶落地水平误差0.087米，base_link地图Z=5.227米。已确认落地并解除锁定，末点仅请求一次降落，完成后3仿真秒不再发目标、心跳或重新解锁。已解锁Offboard阶段机体原点距建筑碰撞盒最小1.861米。

构建与检查通过：49项、0错误、0失败、9跳过，cppcheck 2.7仍按ament规则跳过。当前地图、控制代码、1m/s速度、0.5m/s²加速度、2秒停稳和保护阈值均沿用此前设置。未检查GUI。结果文件：/home/ruan/uav_ws/run/free_route_validation_result.json；本次临时日志：/tmp/uav_free_route_20261111_szwsvj57。

## 9. 路线订阅、发布与A*输入转换

planner_node本来就是路线订阅节点，订阅/uav/mission，类型为nav_msgs/msg/Path。A*源码没有固定P2/P3/P4路线；过去看起来固定，是因为命令一直发布同一个YAML示例。

新增src/mission_publisher_node.cpp提供完整C++发布示例。它从唯一输入waypoints读取[x1,y1,z1,x2,y2,z2,...]，把每组坐标装成一个PoseStamped，依序加入Path.poses，并设置header.frame_id=map。全部点组装完后一次发布，不是每个点单独发布一条任务。

转换过程如下：

~~~text
你的路线发布节点（任意数量、任意顺序的map访问点）
  → /uav/mission：nav_msgs/Path
  → planner_node::on_mission()
  → 读取poses中的x/y/z，保存到anchors_（vector<Point3D>）
  → 起点=/uav/estimated_pose的位置 + 相对起飞高度
  → 对每个访问点调用plan(当前段起点, 访问点)
  → 世界米制坐标按边界原点和分辨率转换成三维栅格索引
  → A*搜索、恢复米制路径、段内简化
  → 全部段成功后发布/uav/path给执行节点
~~~

例如P2的map坐标(-6,15.959,10)，在边界原点(-26,-20,1)、分辨率0.5米时，对应栅格索引(40,72,18)。输入给A*仍使用米制坐标，索引换算由astar_3d.cpp内部完成。ENU→NED转换由飞行执行节点负责，不要在任务发布节点提前转换成NED。

自己的任务分配节点可以直接向/uav/mission发布同样的Path，不必经过mission_publisher_node。新节点只是便于运行和阅读的示例，没有写死目标点坐标；完整可编译的消息组装与publish写法见它的源码。

waypoints参数在启动时提供、启动后只读。该发布节点只发一次；当前规划/飞行节点一轮只接受一个任务，飞行中替换或追加路线尚未实现。下一轮仍需重启整套场景和节点，让飞机回到已标定的起飞点。

2026-11-11已完成构建与检查：51项、0错误、0失败、10跳过，cppcheck 2.7按ament规则跳过。另在独立ROS domain中用P2→P4两个点验证：发布节点只发一次，消息保持点数/坐标/顺序，实际planner_node成功生成包含两点的有序执行路径。此次没有启动控制节点、PX4或Gazebo，没有重复实飞。验证结果保存在/home/ruan/uav_ws/run/mission_publisher_validation_result.json。

## 10. 名称映射与往返测试：S1 → C → S1 → A → S1

名称映射保存在config/mission_points.yaml。表中坐标是map/ENU下的地面或屋顶表面位置，单位米；发布节点读取visit_height=2.0，将表面Z加2米转换成飞行访问高度。

| 名称 | 对应位置 | 表面坐标(X,Y,Z)，米 | 发布的访问高度Z，米 |
| --- | --- | --- | ---: |
| S1 | 第一个停机位 | (-24,-18,0) | 2 |
| S2 | 第二个停机位 | (-22,-18,0) | 2 |
| S3 | 第三个停机位 | (-23,-16,0) | 2 |
| A | P1 | (-23,5.207,5) | 7 |
| B | P2 | (-6,15.959,8) | 10 |
| C | P3 | (16,17,12) | 14 |
| D | P4 | (24,4,5) | 7 |
| E | P5 | (21,-16,8) | 10 |
| F | P6 | (4,-19,12) | 14 |
| G | P7 | (-10,0,0) | 2 |

这里的A明确表示P1，不是旧地图绿色“A基地”模型A_base_visual_only的中心(-23,-17)。S1/S2/S3对应绿色基地内三个白色停机位；它们只有visual标记，实际接触面仍是地图地面Z=0。

### 终端4：发布名称路线

修改C++后先按第8节构建。前三个终端仍按第8节启动仿真、goto_point_node和planner_node；第四个终端选择下面的命名路线命令，不同时发送旧waypoints或YAML示例任务：

~~~bash
source /home/ruan/uav_ws/install/setup.bash
ros2 run uav_planning mission_publisher_node --ros-args \
  --params-file /home/ruan/uav_ws/src/uav_planning/config/mission_points.yaml \
  -p "route:=['S1','C','S1','A','S1']"
~~~

route与旧waypoints参数二选一。route使用名称并查表增加visit_height；waypoints直接提供最终飞行访问坐标，不额外增加高度。名称或坐标配置有误时，发布节点拒绝整条任务，不会只发送前半段。

单机默认命名路线必须以S1开头。首个S1仅表示出发位置，不重复发布为目的点；实际A*起点仍由/uav/estimated_pose和初始相对起飞高度计算。上述5个名称最终转换为4个有序目的点：

~~~text
C  (16,17,14)
  → S1 (-24,-18,2)
  → A  (-23,5.207,7)
  → S1 (-24,-18,2)
~~~

这4个点组成同一条nav_msgs/Path消息，发布到/uav/mission。两次返回S1都会保留，A*在各段之间自动生成绕障路径，然后由/uav/path交给飞行节点执行。

### 本次往返动作的含义

1. 飞机从实际S1起飞，初始机体base_link地图高度约0.227米，相对上升2米后约为2.227米，再按A*路径访问C。
2. 从C返回S1的访问点(-24,-18,2)，停稳0.5秒后继续去A；这次返回S1保持在空中，不降落、不解除锁定。
3. 访问A后再次返回S1的访问点，停稳后由PX4在地面自主降落，确认落地及解除锁定后结束整条任务。

S1表面Z=0、访问Z=2、初始起飞悬停Z约2.227、最终地面落地base_link的Z约0.227，各自含义不同。不能把表面Z=0直接作为A*目的点，也不能使用原P4屋顶的5.227米作为本次落地判据。

本次中途返回S1只是短暂悬停测试，没有模拟真正落地、补货、再次解锁起飞。若需要每次回S1都实际落地补货再起飞，应另行增加对应任务阶段，不能只靠修改名称序列实现。

本节的单机流程只控制S1的x500_0。更改路线中的首个标签不会改变飞机的出生位置；要控制S2的x500_1，使用第11节的双机流程。

2026-11-11验证完成：命名消息在独立域核对为[C14米,S1 2米,A7米,S1 2米]四个目的点，只省略首个起点S1，两次返回均保留。未知名称与错误起点在发布前拒绝，原waypoints坐标发布接口回归通过。构建及检查为51项、0错误、0失败、10跳过；cppcheck 2.7按ament规则跳过。

实际使用上述命名命令跑通完整往返。A*生成10个执行航点，C、第一次S1、A、最后S1分别是第3/6/8/10点；路径长183.88米，含6段斜飞，任务约249.88仿真秒。参考速度峰值1.000m/s，实测瞬态峰值1.188m/s，实际最小建筑净空1.929米。

第一次返回S1时base_link真值约(-24.012,-17.978,2.009)，停稳后继续前往A，没有降落。最终返回S1访问点后只请求一次降落，确认地面落地及解除锁定；最终真值约(-23.969,-17.881,0.227)，水平落点误差0.122米。完成后3仿真秒无新目标、心跳或重新解锁。

首轮验证中，输入监听器先匹配到单次发布节点，规划器尚未完成匹配，导致任务未送到规划器且未起飞。随后把映射消息验证与飞行验证分开：实飞时/uav/mission只由规划器接收，所有状态和输出观测仍保留。完整实跑通过；未修改通用话题接口、A*、控制器或阈值，未检查GUI。

映射消息结果：/home/ruan/uav_ws/run/named_mapping_validation_result.json。
实跑结果：/home/ruan/uav_ws/run/named_route_validation_result.json；本次日志：/tmp/uav_named_route_20261111_qgf9juqr。测试已只清理自有进程。

## 11. 双机演示：S1 和 S2 各自规划、执行

双机 launch 在同一个洪水地图中启动两架 X500。两架飞机各有一个 PX4 实例、
一个规划节点和一个执行节点；Gazebo、地图、DDS Agent 和仿真时钟共用。

| 飞机 | 出生点 (map/ENU，m) | Gazebo 模型 | PX4 instance / 系统 ID | 任务话题 | A* 输出 |
| --- | --- | --- | --- | --- | --- |
| UAV 1 | S1 (-24, -18) | x500_0 | 0 / 1 | /uav1/mission | /uav1/path |
| UAV 2 | S2 (-22, -18) | x500_1 | 1 / 2 | /uav2/mission | /uav2/path |

### 构建与启动

先退出上一轮仿真。终端 1：

~~~bash
cd ~/uav_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select uav_planning --symlink-install
source install/setup.bash
ros2 launch uav_planning dual_uav.launch.py
# 无界面时，改用 ros2 launch uav_planning dual_uav.launch.py gui:=false
~~~

两架飞机都在地面等待任务。日志出现两次“锁定NED参考”后，再发对应的路线。
需要观察时，可以用 ros2 topic list 查看 /uav1 和 /uav2 下的话题；
/uav1/estimated_pose 和 /uav2/estimated_pose 是各自的 map 位置估计。

### 按飞机发布命名路线

终端 2：让 S2 的 UAV 2 去 G（P7）后返回 S2 自主降落。

~~~bash
source ~/uav_ws/install/setup.bash
ros2 run uav_planning mission_publisher_node --ros-args \
  --params-file ~/uav_ws/src/uav_planning/config/mission_points.yaml \
  -r __ns:=/uav2 -r /uav/mission:=/uav2/mission \
  -p start_name:=S2 -p "route:=['S2','G','S2']"
~~~

终端 3：让 S1 的 UAV 1 去 A（P1）后返回 S1 自主降落。
可等 UAV 2 已离开停机位再执行此命令，观察两架同时飞行。

~~~bash
source ~/uav_ws/install/setup.bash
ros2 run uav_planning mission_publisher_node --ros-args \
  --params-file ~/uav_ws/src/uav_planning/config/mission_points.yaml \
  -r __ns:=/uav1 -r /uav/mission:=/uav1/mission \
  -p "route:=['S1','A','S1']"
~~~

start_name 默认 S1，S2 的发布命令指定 S2。首个标签只用来核对出发点，
后续标签才会作为 nav_msgs/Path 中的访问点发布。每架 A* 从自己的预计起飞点
开始，按顺序绕过静态建筑；中途访问点停稳后继续，最后一站才降落。
每架节点每轮接收一条任务，下一轮请重启双机仿真。

任务分配程序可以直接向对应的 /uav1/mission 或 /uav2/mission 发布
map/ENU 坐标的 nav_msgs/msg/Path。每架的 PX4 反馈和命令分别位于
/uav1/fmu 与 /uav2/fmu；Gazebo 真值是 /uav1/ground_truth 与 /uav2/ground_truth，
用于对照各自的位置和落点。A* 不把另一架飞机当作障碍，两条路线应在演示前错开。

2026-11-12 无界面仿真已验证上述两条路线。只派发 S2 任务时，S1 保持在地面上锁；
随后两架同时处于 Offboard 飞行 39.8 仿真秒。
两架各完成 4 个 A* 执行航点并返回各自停机位降落上锁，水平落点误差分别为
0.087 m（S1）和 0.096 m（S2）。
构建与包内检查通过（0 失败、10 跳过）。验证数据见 ~/uav_ws/run/dual_uav_validation_result.json。

## 12. 自动分发：向双机发送一串任务

使用第 11 节的双机 launch。它同时启动 mission_dispatcher_node，默认没有任务，
不会自行起飞。在另一个 WSL 终端发布：

~~~bash
source ~/uav_ws/install/setup.bash
ros2 topic pub --once /missions std_msgs/msg/String "{data: 'ACACABCCB'}"
~~~

data 是 A–G 的连续大写字母，每个字母代表一个目标。可以在两机准备期间先发送；
节点会等飞机就绪再派单。不需要填 S1 或 S2，不加箭头、空格或逗号。
每次发布是一批新任务，追加到现有队列；重复发布同一串也会新增一批。

ACACABCCB 拆成 #1 A、#2 C、#3 A、#4 C、#5 A、#6 B、#7 C、#8 C、#9 B。
分发节点沿用 mission_points.yaml 的坐标，为获派的飞机生成两个关键点：
目标上方的访问点、该机自己的停机位上方的访问点。UAV1 回 S1，UAV2 回 S2，
最后由 PX4 执行降落。中途访问目标时按原有逻辑悬停；返航上锁后即可接下一单，暂不计补货用时。

- 派发按队列顺序，哪架就绪就用哪架。刚派单即占用飞机，无需等它起飞。
- 同一目标允许两架飞机并发接单，两机 A* 各自规划。
- 执行节点核对本单航点完成、最新 PX4 已落地/上锁且距本机停机位不超过 0.5 m（水平），才能报告成功并接下一单。降落高度由 PX4 的落地反馈确认。
- 规划失败、执行失败或反馈超时时，该机停止接单，失败任务不会被自动重发。
- 手动路线（第 11 节）和自动分发请分开轮次；换方式前重启双机 launch。

调试时看 launch 日志里的“收到”“派单”“完成”。也可以在终端查看一架飞机的反馈：

~~~bash
ros2 topic echo /uav2/execution_state
~~~

内部以 task_id 区分重复目标。/uavN/sortie 是带编号的任务关键点，
/uavN/sortie_path 是 A* 输出的同一架次航点，/uavN/execution_state 是执行状态。
READY(0) 表示初始化完成，BUSY 表示本单已接收，SUCCEEDED 表示上次返航完成且可接下一单，
FAILED 表示故障。N 替换为 1 或 2。

已验证：用模拟状态反馈向分发节点发送完整 ACACABCCB，九单按顺序各派发一次，
非法输入整批拒绝；用实际 A* 验证 S1、S2 往返 A/B/C 的六种路线均可规划。
Gazebo/PX4 无界面联调使用 GAG 完成三单，其中 UAV1 降落上锁后再次领单、起飞、
返航并上锁。九单序列未在 Gazebo 中整批实飞。构建和包内检查通过。
验证记录见 ~/uav_ws/run/sortie_dispatch_fake_result.json 和 sortie_dispatch_live_result.json。

## 13. 从独立分配结果启动双机任务

分配模块和 ROS 2 分开运行。双机 launch 仍用第 11 节的启动命令；启动本身不触发飞行。
已有分配结果时，在另一个 WSL 终端读取 JSON 并发布一次：

~~~bash
source ~/uav_ws/install/setup.bash
ros2 run uav_planning allocation_publisher --ros-args -p "results_file:=$HOME/uav_ws/src/uav_planning/distribute/module04_ai_results.json"
~~~

运行工作区里的 distribute/module04_train.py 会在脚本旁生成 JSON。results_file
应指向那次运行生成的文件。若在 D 盘的 Py-project 里运行脚本，则使用
/mnt/d/Py-project/module04_ai_results.json。

发布节点读取 QLearning.rows 里的 drone 和 hub，把每一行转换为「机号 + 目标」
并一次发布到 /assigned_missions（std_msgs/msg/String）。例如，当前工作区结果
ACBAACCCB 对应 1A2C1B2A1A2C1C2C1B。

| 飞机 | 停机位 | 按顺序执行的目标 |
| --- | --- | --- |
| UAV1 | S1 | A → B → A → C → B |
| UAV2 | S2 | C → A → C → C |

需要单独测试新结果时，重启双机 launch 后发布一次。运行中发布会把新任务追加到现有队列。
若发布端提示 DDS 确认超时，先检查双机终端是否打印“收到分配结果”。看到这条日志后，该批任务已入队。

分发节点保留指定机号和每架飞机的内部顺序。每单仍是起飞、访问目标、返回
自己的停机位、落地上锁。确认该机重新就绪后才派发它的下一单。另一架未就绪时，
本机可以先执行自己的任务。两机的下一单目标相同时也可以同时派发。
JSON 的 start、arrival、finish 是分配模型估计，不是 Gazebo 的起飞定时指令。
A* 航点和执行反馈仍由原有 /uavN/sortie_path、/uavN/execution_state 提供。

/missions 的纯字母串仍可做不指定机号的演示。执行已分配结果时，用本节命令，
无需再向 /missions 发送相同的字母串；每次发布都会新增一批架次。

验证：使用原始 JSON 和模拟飞行反馈，九单的机号和各机顺序全部保留。
Gazebo/PX4 使用三单 JSON（2G1A2G）验证从文件发布、指定派单、A*、起飞、
返航落地上锁到同一飞机再次起飞的完整链路。原始九单未在 Gazebo 中整批飞行。
验证记录见 ~/uav_ws/run/assigned_dispatch_fake_result.json 和 assigned_dispatch_live_result.json。

## 14. 返航悬停后接下一单

用这个版本测试双机返航后保持 Offboard，等待下一单。首次收到任务时仍从 S1、S2
起飞。飞完每单后，在各自停机位上方的返航航点稳定 0.5 秒，报告 SUCCEEDED 并继续
发送悬停目标。下一单的 A* 从当前悬停参考位置规划，飞机直接开始飞航点。

~~~bash
cd ~/uav_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select uav_planning --symlink-install
source install/setup.bash
ros2 launch uav_planning dual_uav.launch.py return_hover:=true
~~~

另一个 WSL 终端可以用第 13 节发布分配结果，也可用三单命令单独验证：

~~~bash
source ~/uav_ws/install/setup.bash
ros2 topic pub --once /assigned_missions std_msgs/msg/String "{data: '1G2A1G'}"
~~~

UAV1 连续执行两趟 G，UAV2 执行一趟 A。观察完成日志中的“悬停”，以及 UAV1
第二单直接开始航点跟踪。任务队列清空后飞机继续悬停，可继续发任务。
想复测落地再起飞的版本，重启双机 launch，使用默认的 return_hover:=false。

这个试验跳过架次之间的落地与重新解锁。首次起飞、航点飞行和等待悬停仍检查
0.75 m 跟踪误差，超限会触发原有的停止和应急降落。

Gazebo/PX4 已验证 1G2C1G2A 四单全部完成，包括 UAV2 从 C 返航悬停后直接接 A。
两架各解锁一次，四单均未触发 0.75 m 保护，完成后持续发送悬停目标与 Offboard 心跳。
本轮验证日志在 /tmp/uav_return_hover_2tjgvvqr/launch.log。

## 15. 在 Gazebo 显示规划路径和实际航迹

构建工作区后，在双机启动命令中加上 show_paths:=true，保留 Gazebo 界面。
可以继续使用第 12 节或第 13 节的命令发布任务。

~~~bash
source ~/uav_ws/install/setup.bash
ros2 launch uav_planning dual_uav.launch.py show_paths:=true
~~~

| 无人机 | 当前 A* 规划路径 | 实际飞行航迹 |
| --- | --- | --- |
| UAV1 | 青色 | 亮绿色 |
| UAV2 | 橙色 | 品红色 |

规划线从 A* 的起点画到终点，下一次规划时更新。航迹线由 Gazebo 真值位置累积，
随飞行延长。航迹每移动约 0.2 米取一个点，每架保留最近 3000 个点。
返航悬停版可以同时传入 return_hover:=true。

本功能使用 Gazebo Harmonic 的 MarkerManager 和 Ubuntu 软件包
python3-gz-msgs10、python3-gz-transport13。若切换自定义地图，
可参照 flood_demo.sdf 的 GUI 配置加载 MarkerManager。
