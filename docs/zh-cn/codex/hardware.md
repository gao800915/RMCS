# WheelLegInfantry 迁移意见

## 对照范围
- 当前待迁移文件: `rmcs_ws/src/rmcs_core/src/hardware/wheelleg-infantry.cpp`
- core cpu 迁移参考: `/home/gao/project/RMCS/rmcs_ws/src/rmcs_core/src/hardware/steering-infantry.cpp`
- 单板参考: `/home/gao/project/RMCS/rmcs_ws/src/rmcs_core/src/hardware/omni_infantry.cpp`

## 结论
我的判断是，`wheelleg-infantry.cpp` 不能只做几处 API 替换。要跟上新的 core cpu 改动，需要把它当成一次完整的硬件接入迁移来做，至少要重写下面四层:

1. `CBoard` 接入方式
2. CAN/UART 发包与收包方式
3. 板卡参数和配置项
4. wheelleg 特有的髋关节 `DmMotor` 链路

另外，这个文件本身还有几个和迁移无关、但迁移时必须一起收掉的问题:

- `TopBoard` 类完整存在，但实例化被注释掉了，`update()` 和 `command_update()` 也只跑 `bottom_board_`。
- `gimbal_calibrate_subscription_callback()` 现在是空实现。
- `BottomBoard` 里虽然定义了 `gimbal_yaw_motor_` 和 `bullet_feeder_motor_`，但 `command_update()` / `can*_receive_callback()` 并没有真正处理它们。
- `tf_` 在 board 内被使用了，但构造函数里没有 `register_output("/tf", tf_)`。

这说明 wheelleg 当前代码已经处于“半停用、半保留”的状态。迁移前必须先把实际硬件拓扑说清楚，否则很容易把旧问题一起搬进新框架。

## 必须重写的部分

### 1. `client::CBoard` 要迁到 `agent::CBoard`
`steering-infantry.cpp` 的新实现已经把:

- `<librmcs/client/cboard.hpp>` 换成 `<librmcs/agent/c_board.hpp>`
- 板卡构造参数从 `usb_pid_*` 改成 `board_serial_*`
- 回调签名从“原始参数列表”改成 `librmcs::data::*View`

所以 `wheelleg-infantry.cpp` 里的 `TopBoard` / `BottomBoard` 也应该同样迁移。

对应要改的点:

- 构造函数参数从 `int usb_pid` 改成 `std::string_view board_serial`
- 创建板卡时从 `get_parameter("board_serial_top_board").as_string()` / `get_parameter("board_serial_bottom_board").as_string()` 取值
- `can1_receive_callback` / `can2_receive_callback` / `uart1_receive_callback` / `dbus_receive_callback` / `accelerometer_receive_callback` / `gyroscope_receive_callback` 改成新的 view 风格
- 删除手动 `handle_events()` 线程和 `TransmitBuffer`

### 2. 发包方式要从 `TransmitBuffer` 改成 `start_transmit()`
旧 wheelleg 现在还是这种风格:

```cpp
transmit_buffer_.add_can1_transmission(...);
transmit_buffer_.add_can2_transmission(...);
transmit_buffer_.trigger_transmission();
```

新 core cpu 参考实现已经统一成:

```cpp
auto builder = start_transmit();
builder.can1_transmit({...});
builder.can2_transmit({...});
```

对 wheelleg 来说，下面几类包都要重写:

- 轮毂电机打包
- 髋关节 DM 电机 0x01~0x04 的独立 CAN 帧
- 云台 pitch / yaw
- 弹丸拨弹和摩擦轮
- 裁判系统 UART1 写回

`DJI` / `LK` 这两类电机在新代码里通常用 `device::CanPacket8` 组包；`DmMotor` 因为参考仓库里没有现成迁移版本，我建议也统一成“返回 8 字节 CAN 数据”再交给 `builder.can*_transmit()`，不要继续保留旧的 `TransmitBuffer` 写法。

### 3. 回包接口要改成 `std::span` / `data view`
参考 `steering-infantry.cpp` 和 `omni_infantry.cpp`，新的收包路径不再直接拿 `uint64_t can_data` 或 `const std::byte* + len`。

wheelleg 应该同步改成:

- `can*_receive_callback(const librmcs::data::CanDataView& data)`
- `uart1_receive_callback(const librmcs::data::UartDataView& data)`
- `dbus_receive_callback(const librmcs::data::UartDataView& data)`
- `accelerometer_receive_callback(const librmcs::data::AccelerometerDataView& data)`
- `gyroscope_receive_callback(const librmcs::data::GyroscopeDataView& data)`

然后把原先的:

- `store_status(can_data)` 改成 `store_status(data.can_data)` 或对应的 span/字节适配
- `dr16_.store_status(uart_data, uart_data_length)` 改成 `dr16_.store_status(data.uart_data.data(), data.uart_data.size())`

### 4. 参数名要从 `usb_pid_*` 改成 `board_serial_*`
这个改动不只在 C++ 文件里，`rmcs_ws/src/rmcs_bringup/config/wheelleg-infantry.yaml` 也要一起改。

当前 wheelleg 配置里还是:

- `usb_pid_top_board`
- `usb_pid_bottom_board`

如果沿用双板结构，就应该改成:

- `board_serial_top_board`
- `board_serial_bottom_board`

如果最终确认 wheelleg 已经是单板结构，那就应该像 `omni_infantry.cpp` 一样直接改成单个 `board_serial`，同时删掉多余的 `TopBoard` 逻辑。

### 5. `RingBuffer` 和 UART 读写要一起迁
旧代码使用的是 `librmcs::utility::RingBuffer` 和 `pop_front_multi` / `emplace_back_multi` 风格。

新参考实现已经换成:

- `rmcs_utility::RingBuffer<std::byte>`
- `pop_front_n(...)`
- `emplace_back_n(...)`
- `uart1_transmit({ .uart_data = std::span<const std::byte>{buffer, size} })`

如果 wheelleg 还要保留裁判系统串口，这一块也应该完全按新接口改。

## Wheelleg 特有的重写点

### 1. 先决定 wheelleg 到底是双板还是单板
这是最关键的一步。

从当前 `wheelleg-infantry.cpp` 看，代码的意图明显是“双板”:

- `TopBoard` 负责 `gimbal_pitch_motor_` 和两路摩擦轮
- `BottomBoard` 负责底盘轮毂、四个髋关节、云台 yaw、拨弹、电调、IMU、遥控、裁判系统

但实际运行路径又是“半单板”:

- `top_board_` 被注释掉
- `gimbal_calibrate_subscription_callback()` 被注释空了
- `bottom_board_->command_update()` 根本没有给 yaw / bullet feeder 发控制

所以迁移前必须先选一个方向:

1. 如果真实硬件还是双板:
   参考新的 `steering-infantry.cpp`，把 `TopBoard` 和 `BottomBoard` 都完整迁走，并恢复 `top_board_->update()` / `top_board_->command_update()` / 云台校准路径。
2. 如果真实硬件已经改单板:
   不要继续保留这个半废弃的 `TopBoard`。直接删掉它，把还在使用的设备统一收进一个 board，迁移模板优先参考新的 `omni_infantry.cpp`。

我的意见是，不要继续保留现在这种“类还在、实例没了、接口还留着”的中间态。这个状态最容易在迁移后留下隐性死代码。

### 2. `DmMotor` 没有现成参考，髋关节链路需要单独处理
这个点是 wheelleg 相比 steering 最大的额外工作量。

我在 `/home/gao/project/RMCS` 里没有找到新的 `hardware/device/dm_motor.hpp`，也没找到任何新的 `DmMotor` 使用点。也就是说:

- `steering-infantry.cpp` 只能提供板卡框架迁移的模板
- 不能直接覆盖 wheelleg 四个髋关节的迁移

所以 wheelleg 要么:

1. 把当前仓库里的 `hardware/device/dm_motor.hpp` 一起迁过去，并补上新的 span/字节接口
2. 要么按新 `DjiMotor` / `LkMotor` 的写法，重写一个新的 `DmMotor` 包装层

我更倾向于第二种写法，原因是新仓库里的 `DjiMotor` / `LkMotor` 已经不再依赖旧的 `librmcs::device::*` 透传接口，而是自己直接管理字节包和 `as_bytes()`。如果 wheelleg 继续保留旧 `DmMotor` 风格，整份文件会长期处于“两套硬件抽象并存”的状态，后面更难维护。

### 3. `tf_` 必须补注册
当前 wheelleg 里 `TopBoard` / `BottomBoard` 都在使用 `tf_`，但外层构造函数没有像其它硬件文件那样先做:

```cpp
register_output("/tf", tf_);
```

这个问题和 core cpu 迁移无关，但我认为在重写时必须一起修掉。否则即使迁到新框架，`tf_` 这条链路仍然是不完整的。

### 4. IMU 输出命名建议统一，但要连控制器一起改
当前 wheelleg 发布的是:

- `/gimbal/yaw/velocity`
- `/gimbal/pitch/velocity`
- `/chassis/yaw/velocity`
- `/chassis/pitch/velocity`
- `/chassis/roll/velocity`

新 RMCS 里更常见的是 `*_imu` 后缀，例如:

- `/gimbal/yaw/velocity_imu`
- `/gimbal/pitch/velocity_imu`
- `/chassis/yaw/velocity_imu`

这里我给出的意见是:

1. 如果你只是想先把 wheelleg 迁到新 core cpu 并保持现有 wheelleg 控制器可用，可以先保留旧命名。
2. 如果你想把 wheelleg 也并到当前 RMCS 的统一接口风格，就应该把这些输出改成 `*_imu`，但要同步修改:
   - `controller/chassis/wheel_leg/wheel_leg_controller.cpp`
   - `rmcs_ws/src/rmcs_bringup/config/wheelleg-infantry.yaml`
   - 任何依赖这些 topic 的调试/广播组件

也就是说，这一项不是单文件重写，必须和控制器一起做。

## 不要直接照抄 steering 的地方
`steering-infantry.cpp` 只能照抄“迁移方法”，不能照抄“设备分配和 CAN 细节”。

特别是下面这些点，wheelleg 必须保留自己的硬件语义:

- wheelleg 是两路轮毂电机，不是四路转向轮电机
- wheelleg 的四个髋关节是 `DmMotor`，不是 `GM6020`
- 当前 wheelleg 的 hip CAN ID 是 `0x01`~`0x04`，这和 steering 的 steer 电机回包/发包 ID 完全不是一套
- 当前 wheelleg 的云台 pitch 用的是 `MG5010`，不要直接照抄 steering 的 `MG4010`
- 当前 wheelleg 控制器还依赖 `/chassis/yaw/velocity` 这一类旧命名

所以我的建议是:

- 框架层照着新的 steering/omni 写
- 电机类型、CAN ID、topic 名字、是否保留 top board，全部按 wheelleg 自己的硬件重新整理

## 推荐重写顺序
我建议按这个顺序做，风险最低:

1. 先确定 wheelleg 是双板还是单板，同时决定 gimbal/shooting 路径是不是还要保留。
2. 把外层类和 board 类迁到 `agent::CBoard`，先把 `board_serial_*`、回调签名、`start_transmit()`、UART/RingBuffer 改完。
3. 先迁底盘轮毂、IMU、DR16、裁判系统，让 chassis 主链路先跑通。
4. 单独补 `DmMotor` 迁移，再接四个 hip 的发包和回包。
5. 最后再决定是恢复 `TopBoard`，还是彻底删除 `TopBoard` 并清理云台/拨弹死代码。
6. 如果要统一接口命名，再一起修改 wheelleg 控制器和 YAML。

## 最后的判断
如果只看“core cpu 接口变更”，wheelleg 的必改项其实已经很明确了: `CBoard`、回调、发包、参数、UART/RingBuffer 都要按新的 steering 方式重写。

但如果看“这份 wheelleg 文件本身现在的状态”，真正更重要的是先做结构决策:

- `TopBoard` 要不要继续存在
- `gimbal_yaw_motor_` / `bullet_feeder_motor_` 是不是还要实际工作
- `DmMotor` 是继续沿用旧包装，还是一起升级成新的字节包风格

我的意见很明确: 这次不要做“最小替换式迁移”，而是顺手把这几个结构问题一次理干净。否则你最后得到的会是一个已经换成新 core cpu 接口、但内部仍然是半废弃状态的 `wheelleg-infantry.cpp`。
