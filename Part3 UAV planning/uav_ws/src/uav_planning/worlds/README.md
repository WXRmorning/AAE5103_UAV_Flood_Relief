# 洪灾配送地图：Harmonic + PX4 X500

场景文件：/home/ruan/uav_ws/src/uav_planning/worlds/flood_demo.sdf。
使用 SDF 1.9，Gazebo 世界名 flood_demo，机体名 x500_0。
官方模型来自 /home/ruan/PX4-Autopilot/Tools/simulation/gz/models/x500，
其中包含四个电机、机体和传感器。PX4 的本地估计和底层控制通过 Gazebo 接口完成。

## 推荐启动

~~~bash
source /home/ruan/uav_ws/install/setup.bash
ros2 launch uav_planning takeoff_hold.launch.py target_altitude:=2.0
~~~

只看地图和仿真飞机，暂不起飞：

~~~bash
ros2 launch uav_planning takeoff_hold.launch.py start_controller:=false
~~~

launch 自动设置模型路径并使用 Ogre1。无界面加 gui:=false。
不要再使用旧 Fortress 的 ign gazebo 命令。

## 只用 Gazebo 打开地图

~~~bash
export GZ_SIM_RESOURCE_PATH=/home/ruan/PX4-Autopilot/Tools/simulation/gz/models
export GZ_SIM_SERVER_CONFIG_PATH=/home/ruan/uav_ws/src/uav_planning/config/gazebo_server.config
gz sim -r --render-engine ogre /home/ruan/uav_ws/src/uav_planning/worlds/flood_demo.sdf
~~~

该方式只启动物理场景，没有 PX4 和 ROS 控制程序，不会自动起飞。
如果之后要启动飞控，应关闭这个实例后用 launch，避免重复实例和同名模型。

世界系统集中在 config/gazebo_server.config：物理、模型操作、场景广播、
IMU、气压计、GPS、磁力计等。SDF 不重复声明部分世界系统，
防止自定义 world 覆盖默认系统后缺失传感器更新。
当前没有相机推流插件，不需要 GstCameraSystem 动态库。

世界另设置了与 PX4 官方默认场景相同的 WGS84 参考经纬度，
为 GPS/地磁估计提供地理参考，不改变建筑和任务点的本地米制坐标。

## 尺度和场景坐标

场地 60×50 米，X=[-28.5,31.5]、Y=[-22.5,27.5]，地面顶面 Z=0。
蓝色水面覆盖其余场地，西南角保留 12×12 米方形起降区，范围为 X=[-28.5,-16.5]、Y=[-22.5,-10.5]。绿色基地中心仍为 (-23,-17)。
重力为 -9.81 m/s²，仿真步长0.001秒。X500 保持官方原尺寸，不随地图缩放。

| 建筑 | 中心 X | 中心 Y | X 尺寸 | Y 尺寸 | 高度 |
| --- | ---: | ---: | ---: | ---: | ---: |
| B1 | -16 | -1 | 5 | 12 | 6 |
| B2 | -3.5 | 4.5 | 7 | 9 | 10 |
| B3 | 9.5 | 5.5 | 6 | 8 | 14 |
| B4 | 6 | -9.5 | 9 | 5 | 8 |
| B5 | -14 | 12 | 5 | 4 | 4 |
| P1配送楼 | -23 | 5.207 | 6 | 6 | 5 |
| P2配送楼 | -6 | 15.959 | 6 | 6 | 8 |
| P3配送楼 | 16 | 17 | 6 | 6 | 12 |
| P4配送楼 | 24 | 4 | 6 | 6 | 5 |
| P5配送楼 | 21 | -16 | 6 | 6 | 8 |
| P6配送楼 | 4 | -19 | 6 | 6 | 12 |

| 标记 | X | Y | 标记模型Z（屋顶或地面） |
| --- | ---: | ---: | ---: |
| 绿色基地中心 | -23 | -17 | 0 |
| P1 | -23 | 5.207 | 5 |
| P2 | -6 | 15.959 | 8 |
| P3 | 16 | 17 | 12 |
| P4 | 24 | 4 | 5 |
| P5 | 21 | -16 | 8 |
| P6 | 4 | -19 | 12 |
| P7 | -10 | 0 | 0 |

P1–P6 的标记位于各自屋顶中心。P1、P2 和对应楼体一起平移，P3 保留原位。
从绿色圆盘中心 (-23,-17,0) 到 P1、P2、P3 屋顶标记的三维直线距离分别为
22.76、37.94、53.11 米，比例约为 1.5:2.5:3.5。A* 从各机停机位上方出发，
访问屋顶上方 2 米并绕开建筑，规划航程和实际飞行耗时与这个几何比例有差异。

黄色小球半径为0.10米，
相对标记原点的球心高度0.25米，所有目标标记都只有visual，没有collision。
橙色圆盘尺寸保持原值；真正承接屋顶降落的是长方体楼的collision。

三个停机标记是 (-24,-18)、(-22,-18)、(-23,-16)。
只有第一处放入 x500_0，生成 pose 为 -24 -18 0.3 0 0 0。
0.3米是初始机体原点高度，受重力沉降后会变化，不是悬停目标。

## SDF 入门和修改建筑

pose 顺序为 x y z roll pitch yaw；模型默认相对世界，link 相对模型，
visual/collision 相对 link。长度用米，角度用弧度。

- static=true：建筑固定。
- visual：显示形状；collision：物理碰撞形状。
- box/size：长方体的X、Y、Z尺寸。
- material：颜色。水面和基地/目标标记只有 visual，不会阻挡飞机。

例如 B1 的 pose 是 -16 -1 3 0 0 0，尺寸为5×12×6。
改成9米高时，visual 和 collision 的高度都要改为9，
模型中心Z改成4.5，保持底面位于Z=0。
只改显示形状会使看到的建筑与碰撞体不一致。

## 与路径规划的关系

当前是已知静态场景，planner_node已读取building_*的pose与box collision，
生成0.5米三维栅格，按1.5米净空加半格余量膨胀后执行26邻域A*。
只支持当前无旋转的static长方体建筑，其他建筑几何会报错，不会静默漏掉障碍。
尚未接入灾区重建、SLAM、电池或真实水体动力学。
地图边界也需要作为飞行约束，水面示意不自动等于禁飞区。

地图使用 ENU，PX4 估计位置使用本地 NED，且两者原点不相同。
航点跟踪通过已有frame_alignment模块处理坐标旋转和起点偏移。
P1–P6的标记Z是屋顶表面，飞行访问点使用屋顶上方2米，不能直接发布屋顶表面Z。
默认示例任务为P2上方10米、P3上方14米、P4上方7米三个访问点，实际由发布节点输入，
不指定16米中转层，段间由A*选择高度和绕行路径，允许斜向爬升/下降。
节点最大参考速度为1.0米/秒，加速度为0.5米/秒²，
再由PX4在最后访问点降落；P2/P3/P4消息示例见config/p2_p3_p4_path.yaml。
旧地图Z=2的P1直飞和P2/P3/P4路径已失效，不能沿用。

/simulation/ground_truth/odometry 为世界坐标真值，仅用于记录和验收。
控制节点使用 PX4 的 /fmu/out/vehicle_local_position 估计，不用真值代替估计器。
楼顶多航点启动和降落步骤见 /home/ruan/uav_ws/src/uav_planning/COMMAND.md第8节。
