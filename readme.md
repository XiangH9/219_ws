# 1. 代码运行方式
1. 编译

* v3.1之后：
```
colcon build   --packages-up-to ocs2_core unitree_guide_controller go1_description keyboard_input hardware_unitree_mujoco   --symlink-install   --event-handlers console_direct+   --continue-on-error   --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=${HOME}/219_ws/install
```
* v3.1之前：
```
colcon build --packages-up-to unitree_guide_controller go1_description keyboard_input hardware_unitree_mujoco --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --event-handlers console_direct+ --continue-on-error --cmake-args -DCMAKE_INSTALL_PREFIX=${HOME}/219_ws/install
```
2. source一下资源目录
```
source install/setup.bash
```
3. 运行launch文件
```
ros2 launch unitree_guide_controller gazebo_classic.launch.py
```
4. 开启plotjugger进行数据可视化
```
LD_PRELOAD=/lib/x86_64-linux-gnu/libpthread.so.0 QT_QPA_PLATFORM=xcb LD_LIBRARY_PATH=/lib/x86_64-linux-gnu:/usr/lib/x86_64-linux-gnu:/opt/ros/humble/lib ros2 run plotjuggler plotjuggler
```


# 1.1 仿真实现分支（Gazebo Classic）
## 目标
这套仓库最初主要是围绕真实机器人控制调试起来的。移植到Gazebo Classic之后，最核心的工作不是重写整套FSM，而是把“真机默认成立”的一些前提补齐成“仿真里也成立”。

目前已经确认：
* `passive -> fixed down -> fixed stand` 站立链路在仿真中已经基本打通
* 之前“站不起来”的主因不是FSM本身坏掉，而是仿真运行时模型参数、执行层限幅、接触工况和目标姿态之间不匹配
* 现在新的主问题已经转移到 `trotting` 步态稳定性，而不是站立状态切换

## 一个非常重要的事实
Gazebo Classic运行时真正使用的是：
* [robot.xacro](/home/xiangh9/xhros2/219_ws/src/quadruped_ros2_control-humble/descriptions/unitree/go1_description/xacro/robot.xacro)
* [const.xacro](/home/xiangh9/xhros2/219_ws/src/quadruped_ros2_control-humble/descriptions/unitree/go1_description/xacro/const.xacro)
* [leg.xacro](/home/xiangh9/xhros2/219_ws/src/quadruped_ros2_control-humble/descriptions/unitree/go1_description/xacro/leg.xacro)
* [gazebo_classic.xacro](/home/xiangh9/xhros2/219_ws/src/quadruped_ros2_control-humble/descriptions/unitree/go1_description/xacro/gazebo_classic.xacro)

而不是平时更容易打开查看的：
* [robot.urdf](/home/xiangh9/xhros2/219_ws/src/quadruped_ros2_control-humble/descriptions/unitree/go1_description/urdf/robot.urdf)

因此仿真站不起来、关节限幅、接触参数、Gazebo执行效果，优先应该检查 `xacro` 链，而不是 `urdf`。

## 从真实机器人控制到仿真状态机切换，我们实际做了什么
### 1. 保证控制器只初始化一次，避免仿真回放消息导致崩溃
涉及文件：
* [UnitreeGuideController.cpp](/home/xiangh9/xhros2/219_ws/src/quadruped_ros2_control-humble/controllers/unitree_guide_controller/src/UnitreeGuideController.cpp)

修改原因：
* 仿真里 `/robot_description` 采用 `transient_local`，可能重复触发回调
* 原来的写法会重复构造 `QuadrupedRobot / BalanceCtrl / ConvexMpcSolver`
* 这会在Gazebo里引发重复初始化、`double free`、`gzserver` 崩溃

当前处理：
* `robot_description` 回调只允许初始化一次
* 初始化成功后直接释放这个订阅器

### 2. 增加悬挂启动能力，用于验证FSM而不是直接受地面接触干扰
涉及文件：
* [gazebo_classic.launch.py](/home/xiangh9/xhros2/219_ws/src/quadruped_ros2_control-humble/controllers/unitree_guide_controller/launch/gazebo_classic.launch.py)

修改原因：
* 真机调试时可以悬挂起步
* 仿真默认是一生成就落地，容易把 `calf` 压在下限附近，导致误判为状态机失效

当前处理：
* 新增 `debug` launch参数
* 传给 `robot.xacro` 的 `DEBUG`
* 利用 `robot.xacro` 中 `world -> base` 的固定关节实现悬挂启动

结论：
* 悬挂实验已经证明 `passive -> fixed down -> fixed stand -> trotting` 这条控制链本身是可运行的
* 说明原始问题主要在落地接触工况，而不是FSM框架本身

### 3. 恢复仿真运行时关节能力上限
涉及文件：
* [const.xacro](/home/xiangh9/xhros2/219_ws/src/quadruped_ros2_control-humble/descriptions/unitree/go1_description/xacro/const.xacro)

关键修改：
* `hip_torque_max: 1.0 -> 23.7`
* `thigh_torque_max: 1.0 -> 23.7`
* `calf_torque_max: 1.0 -> 35.55`

修改原因：
* `219_ws` 运行时关节 effort limit 一度被写成了 `1.0`
* 这会导致上层控制虽然给出了姿态和力矩目标，但Gazebo执行层根本发不出足够的关节能力
* 这是之前“站不起来”的根因之一

### 4. 把站立目标姿态改成适合当前这台仿真机器人，而不是继续沿用原始go1风格目标
涉及文件：
* [UnitreeGuideController.h](/home/xiangh9/xhros2/219_ws/src/quadruped_ros2_control-humble/controllers/unitree_guide_controller/include/unitree_guide_controller/UnitreeGuideController.h)

关键修改：
* `stand_pos_` 调整为四条腿一致的 `{0.0, 0.78, -1.36}`
* `down_pos_` 调整为四条腿一致的 `{0.0, 1.08, -2.02}`

修改原因：
* `219_ws` 的几何参数已经不是原始go1尺度
* 继续沿用旧的 `0.9 / -1.53 / -2.4` 一类目标，会让落地工况下的腿部几何非常不合理
* 重新匹配当前模型后，`fixed down / fixed stand` 才能在Gazebo里收敛

### 5. 保留 `passive` 原始逻辑，不把它硬改成“半站立状态”
涉及文件：
* [StatePassive.cpp](/home/xiangh9/xhros2/219_ws/src/quadruped_ros2_control-humble/libraries/controller_common/src/FSM/StatePassive.cpp)
* [StatePassive.h](/home/xiangh9/xhros2/219_ws/src/quadruped_ros2_control-humble/libraries/controller_common/include/controller_common/FSM/StatePassive.h)

过程说明：
* 调试中一度尝试把 `passive` 改成轻控制预备姿态
* 后续在恢复运行时 torque limit、修正 `stand_pos/down_pos` 之后发现：
  * 即使 `passive` 恢复为原始 `kp=0, kd=1` 的阻尼态
  * `fixed down / fixed stand` 也已经能正常带起机器人

当前结论：
* `passive` 不是导致仿真站不起来的主因
* 当前保留原始逻辑更接近原仓库语义，也更利于和真机版本对照

### 6. Gazebo执行层接触参数做了仿真友好化调整
涉及文件：
* [gazebo_classic.xacro](/home/xiangh9/xhros2/219_ws/src/quadruped_ros2_control-humble/descriptions/unitree/go1_description/xacro/gazebo_classic.xacro)

关键修改：
* `self_collide` 从 `1` 调为 `0`
* 接触刚度 `kp` 从 `1e6` 调为 `1e5`
* 接触阻尼 `kd` 从 `1.0` 调为 `10.0`

修改原因：
* 原始参数更像真机模型或更硬的接触条件
* 在Gazebo Classic里容易导致落地起步时腿部接触过硬、`calf` 被锁死在下限附近

说明：
* 这些修改不是FSM逻辑本身
* 但它们直接决定状态切换时机器人是否能从落地状态顺利进入 `fixed down / fixed stand`

### 7. trotting步态开始做“仿真防发散”处理
涉及文件：
* [StateTrotting.cpp](/home/xiangh9/xhros2/219_ws/src/quadruped_ros2_control-humble/controllers/unitree_guide_controller/src/FSM/StateTrotting.cpp)
* [GaitGenerator.cpp](/home/xiangh9/xhros2/219_ws/src/quadruped_ros2_control-humble/controllers/unitree_guide_controller/src/gait/GaitGenerator.cpp)

当前已经做的修改：
* 在 `calcQQd()` 中给 `thigh/calf` 的 `q_goal/qd_goal` 加了限幅
* 逆解结果出现 `NaN/Inf` 时回退到当前关节位置或零速度
* 把 `GaitGenerator` 里还在使用的旧名义站姿，统一改成新的 `{0.0, 0.78, -1.36}`

修改原因：
* 站立问题解决后，新的主问题转移到了 `trotting`
* 一进入 `trotting`，最先发散的是 `thigh/calf` 的目标轨迹和逆解输出，而不是 `leg_pd_controller` 本身

## 当前仿真分支的状态
### 已经基本确认有效的修改
* `UnitreeGuideController.cpp`：只初始化一次机器人模型
* `const.xacro`：恢复合理 torque limit
* `UnitreeGuideController.h`：更新 `stand_pos_ / down_pos_`
* `gazebo_classic.launch.py`：支持悬挂启动 `debug:=true`
* `gazebo_classic.xacro`：仿真接触参数友好化

### 当前已经不再需要保留的临时思路
* 把 `passive` 改成预备姿态
* 单纯靠提高 spawn 高度模拟悬挂
* 单纯扩大 `urdf/robot.urdf` 里的 effort limit

### 当前主线问题
* `fixed down / fixed stand` 基本已经正常
* `trotting` 仍然存在目标轨迹、逆解和接触切换耦合导致的不稳定

## 当前有效基线
日期：
* 2026.05.27

分支与版本：
* branch: `gazebo-trotting-debug`
* commit: `8e2718f`

当前已经验证更有效的一组组合：
* `stand_pos_`：
```cpp
{0.0, 0.97, -1.67}
```
* `down_pos_`：
```cpp
{0.0, 1.25, -2.70}
```
* Gazebo 接触参数：
  * 恢复原版 `gazebo_classic.xacro`
  * `self_collide = 1`
  * `foot/thigh kp = 1000000.0`
  * `foot/thigh kd = 1.0`

当前观测结论：
* `fixed stand` 已明显优于之前的“持续大滑移”状态
* `thigh/calf dq` 从原先约 `-1.87 / 3.10` 下降到了更小量级
* `trotting` 已从早期发散状态进入“基本可控、可继续整定”的阶段

推荐验证标准：
* `fixed stand` 下：
  * `|thigh dq|` 尽量小于 `0.2`
  * `|calf dq|` 尽量小于 `0.3`
* `trotting` 下：
  * 连续运行 `30s`
  * 不出现明显关节撞极限
  * `support-force` 不出现持续异常飙升
  * `joint-swing/joint-support` 的目标与实际不明显发散

关键文件：
* [UnitreeGuideController.h](/home/xiangh9/xhros2/219_ws/src/quadruped_ros2_control-humble/controllers/unitree_guide_controller/include/unitree_guide_controller/UnitreeGuideController.h)
* [gazebo_classic.xacro](/home/xiangh9/xhros2/219_ws/src/quadruped_ros2_control-humble/descriptions/unitree/go1_description/xacro/gazebo_classic.xacro)
* [StateTrotting.cpp](/home/xiangh9/xhros2/219_ws/src/quadruped_ros2_control-humble/controllers/unitree_guide_controller/src/FSM/StateTrotting.cpp)

## 建议的仿真启动方式
### 普通仿真
```
ros2 launch unitree_guide_controller gazebo_classic.launch.py pkg_description:=go1_description
```

### 悬挂验证FSM
```
ros2 launch unitree_guide_controller gazebo_classic.launch.py pkg_description:=go1_description debug:=true
```

### 启动前建议彻底清理
```
pkill -9 gzserver
pkill -9 gzclient
pkill -9 -f gazebo
pkill -9 -f spawn_entity.py
pkill -9 -f robot_state_publisher
pkill -9 -f controller_manager
pkill -9 -f spawner
pkill -9 -f "ros2 launch"
```


# 2. 四足机器人开发过程记录
## V1系列
**V1系列跑通了所有底层硬件通讯和初始机器人数学建模和姿态调试，并完成至troting状态的正常开环运行**
* 2026.01.14 V1.1 完成了新硬件接口的初步对接！
* 2026.01.16 V1.2 补充了imu的硬件底层（未测试）
  *使用下面指令编译整个工程： colcon build --packages-up-to unitree_guide_controller go1_description keyboard_input hardware_unitree_mujoco --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --event-handlers console_direct+ --continue-on-error --cmake-args -DCMAKE_INSTALL_PREFIX=${HOME}/219_ws/install # 仅加这行，确保路径统一
* 2026.01.17 V1.3 完全完成了imu和电机底层移植 在rviz中显示
* 2026.01.17 V1.4 周博然发现重大问题：热插拔接口之后串口顺序混乱。解决方法：不需要代码作任何修改，不要再热插拔了，插拔一定全部关机即可。但是功率电源可热插拔。
* 2026.01.22 V1.5 机器人可以正常固定站立并且踹不倒，制定不准再热插拔的规则
  * 重大修改1：使用以下命令开启plotjugger进行数据可视化： LD_PRELOAD=/lib/x86_64-linux-gnu/libpthread.so.0 QT_QPA_PLATFORM=xcb LD_LIBRARY_PATH=/lib/x86_64-linux-gnu:/usr/lib/x86_64-linux-gnu:/opt/ros/humble/lib ros2 run plotjuggler plotjuggler
  * 修改2：把所有ttyACM* 全部换成了ttyCAN* ，但是这并没有必要，是一次尝试USB接口热插拔的失败尝试，可忽略也可改回
  * 重大修改3：尝试改变fix stand状态的kp和kd，kp从80→100→120，机器人可以正常固定站立并且踹不倒
  * 发现问题：第一是imu零飘非常严重。第二是四足的挂钩在站立后自动脱落，有很大危险性，并且有毛刺会伤人。
* 2026.01.24 v1.51 尝试了freestand状态，基本成功但是电流不足 机器人立不住。另外串口底层的库总是缺失导致串口初始化无法完成，最终解决方法都是与下方内容有关，具体也没多研究：
  * 强制将/usr/local/lib加到LD_LIBRARY_PATH最前面（覆盖ROS2的默认配置）
  * export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
* 2026.01.24 v1.53 发现了足底打滑的问题导致运动学解算出了问题，调大了2和3状态的kp值到160。
  * free stand状态调试完成，功能全部正常。
* 2026.01.24 v1.54 机器人报废。
  * 小腿连杆全部因冲击断裂。
  * 电机发热，但经测试，髋关节和大腿的电机仍然可正常工作，小腿电机生死未卜，但是大概是没有问题的。
  * 多根电源线/can信号线被拽断，有电路烧糊的味道，猜测是电源转接PCB板温度过高
  * troting状态的调试搁置，整理思路
* 2026.02.06 v1.6
  * 硬件软件再次全部修好
* 2026.04.04 v1.7
  * 修改1：将整体控制系统频率降为500hz（首先在gazebo仿真中验证，与1000hz区别不大，可以正常行走）然而我发现之前其实早就修改成500hz了
  * 重大修改2：StateTrotting.cpp中加入一个标志位：troting_kalman，其值为1时使用卡尔曼滤波的位姿闭环，其值为0时则开环调试。
  * 重大修改3：passive到fixdown状态的平滑启动，不会再有冲击启动了。
  * 重大修改4：troting步态大量调整，加入大量的脱离estimator的开环troting代码，代码基本可以用了，理论上可以站在地上troting，但是实际上应该需要再对准一下电机绝对位置。
* 2026.04.05 v1.8
  * 修改1：优化了补全开环troting的代码
  * 修改2：将整个步态周期时长大幅提升，直接从0.65提升到5.65,发现了即使在过量冗余时间内，四条腿仍然是突然的超调运动，而不是平稳遵循足底摆线，排查问题
  * 重大修改3：第一次跑通开环troting！新加入了大量的开环troting内容。其中有大量需要优化的代码如自己写的二连杆的运动学逆解。
  * 明确了：关节最大扭矩的限幅处，就在HardwareUnitree.cpp文件的注释：修改限幅2026.02.06这里，我改大了一些。
  * 至此，V1系列完结，流程全部跑通。
 
 ## V2系列
**V2系列完成了troting闭环的所有算法验证**
* 2026.04.07 v2.01
  * 修复1：单圈第二编码器导致电机初始化移动位置舍近求远的问题（已实机验证）
* 2026.04.09 v2.02
  * 无实际功能修改，修改大量注释和冗余代码，troting开环整个算法运动学流程总结至飞书
* 2026.04.09 v2.03
  * 成功改掉所有的不标准正逆解,完成了很完美的纯位置的正逆解(还未考虑旋转).足端位置已经有明显的摆线运行轨迹
* 2026.04.12 v2.11
  * 尝试加入MarkerArray库进行轨迹可视化，成功，可在rviz中实时显示100个队列的实时最新的四个足底坐标
* 2026.04.13 v2.2
  * 机器人落地开环troting成功，进行了一定的kp kd调参，机器人基本troting完成了，开始闭环控制。
* 2026.04.13 v2.21
  * 开始闭环调试之前，最后补充了雅可比关节速度逆解,实机测试成功，正式开始闭环控制。
* 2026.04.13 v2.3
  * 完成了第一个闭环：roll/pitch闭环的实机调试。还需要调参数
* 2026.04.14 v2.4
  * 加入了一个新的FixedStandOffsetKeyboard.cpp，键盘控制节点，可以同时与原先的键盘控制节点同时开启，此节点只可以用于fixedstand模式！用于微调四足机器人的标准站立步态！
  * 微调模块很正常，但现在最大的问题是什么样的站姿是正确的？
* 2026.04.15 v2.5
  * 在郭久双的帮助下在solidworks标定物理极限时各关节位置，并完成关节位置的精准标定。现在正式完成关节位置标定的关节位置offset后面会加一个“# accurate”，表示这是绝对精准的真实角度了
* 2026.04.16 v2.61
  * 加入了未完成的足底反力分配算法，但是核心balance_ctrl_->calF函数还没改，而且电机MIT控制的话是没法做到精准力控的，但是改成了目标力乘以衰减系数，然后加到tauff里面实现调节。
* 2026.04.19 v2.62
  * 完成了足底力分配算法核心balance_ctrl_->calF函数QP二次优化问题的学习，暂时禁用roll/pitch闭环，物理参数待更新标定。
  * 足底力分配QP优化算法正常运行，抬腿高度至少要0.05才有用，目前的参数抬腿太低的话后腿是抬不起来的，待调参
  * 重大问题！右前的大腿和小腿，又是同时转了一整圈。我心态崩了。修改逻辑：非常详细地整理底层offset修正逻辑算法，然后加入补丁：思路：因为初始的read函数不断读取的passive状态，各关节总会落在正常的位置，所以多加一轮判断，如果位置差别过大直接while(1)卡死，无法忍受。
* 2026.04.21 v2.62修复
  * 全新的HardwareUnitree.cpp内部底层确定关节位置逻辑，加上硬保险逻辑，进一步保证安全。
* 2026.04.23 v2.7
  * 严谨大量算法，加入完整的定向本体系的概念P，修正之前的一系列误认为是世界系G的变量/注释等。
  * 开始调试里程计
* 2026.04.27 v2.7加强
  * 经过电机基本安全测试之后，仍然怀疑电机异常转动，修改了更严格的电机底层安全代码。
  * 再次经过数次测试，电机再没有出现故障。
* 2026.04.28 v2.7再加强
  * UnitreeGuideController.cpp状态机切换代码关键修改：把状态机的更新放在最后，否则会漏掉一拍新状态
  * 这么一个不起眼的小修改竟然非常非常起作用！所有状态切换都变得无比平滑，否则之前都有很严重的位置突变尖峰
* 2026.04.29 v2.8
  * 因为一个机器人总有最适合自己重心的初始姿态，所以我也是暂时放下了执拗，不再追求完全标准的站立姿态，而是调整到了一个四条腿力分配最均匀的姿态，同时注意此时的标准期望pitch就不是0了，而是0.05。调整之后的开环troting非常好，机身不断往后退的问题完全解决。
  * 解决剧烈左右摇摆和机身高度剧烈颠簸的方法也有了，就是要把每次步态开始足底位置换为上一次的结束位置。但是这个就没法再靠B系或P系了，必须上G系。所以一定要调里程计了。
* 2026.05.01 v2.9
  * 里程计在第一日就已经解决，可以看到比较迟钝的机身位置和速度。 
  * 为期三天的超大体量修改。
  * 可以实现最长5秒的脱离绳子的troting完全闭环原地踏步。
  * 瓶颈卡在gait步态生成阶段了，总是走着走着步态越来越离谱了导致失稳，究其原因，足端的end_p每次是不应该等于start_p的，而是应该另有算法，原作者已经写了算法，还没看，实在撑不住了。
* 2026.05.05 v2.10
  * 闭环troting所有算法验证完成！机器人可以离绳闭环troting 10秒，所以说明各种运动学/动力学控制器，卡尔曼滤波里程计算法，步态生成和修正算法全部正常，剩下的等待调参。
  * 闭环troting虽然没有完全落地，但是此阶段的任务已经完全完成，troting已经没有问题。
  * 至此，V2系列完结，流程全部跑通。
 ## V3系列
**V3系列开发中**
* 2026.05.05 v3.00
  * 现在的情况是所有的闭环都正常，但整体都不稳，找不到明显的突破点
  * 首先加上摆动腿的闭环轨迹跟踪，但是效果不好，所以参数调小
  * 现在仅剩最后一个但是最重要的一项：将髋关节闭环加入调参，之前髋关节受到极其严格的限位保护，但是现在基本没有剩下没写的算法了，所以足以加入也必须加入，现在足底力分配的髋关节部分还没放开。
  * QP只能解决中小的机身位置和旋转误差，如果发生巨大偏移，QP不其作用甚至会损坏电机，需要步态修正
  * 今天是里程碑，四足第一次走得非常好。
* 2026.05.07 v3.01
  * 几乎完全优化了当前troting代码并重新测试无误，代码可供大家学习使用
  * 增大了许久未动的机身位置PD控制的D项，效果出奇地好，进一步调参应该参考pd调节理论，待整理
  * 今天出现了完全一样的参数，但是下午和晚上两次运行却不同的情况。究其原因：确定是imu的yaw轴零漂，经过gpt的定性推导，yaw轴零漂会导致QP分配出现错误的Fy，现象完全对上了。所以下一步要在里程计下手了。
  * 后来发现是imu上电初始化期间是不允许运动的，必须静止。这也算是学艺不精，踩坑了，并且imu用久了或者初次买来需要磁力计校准。
* 2026.05.14 v3.1
  * 重大更新：初次移植加入一整套OSC2库，和高仿的 基础convex MPC的代码，更新了colcon build的语句。在libraries目录里加了全套的ocs2_ros2的第三方库
  * 弃用了原来的QP++的求解库，它已经不适合高速QP求解了，改用ocs2_ros2库自带并且性能足够优秀的HPIPM库。
