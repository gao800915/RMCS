# `dm_motor.hpp` 需要怎么改

## 直接结论

`rmcs_ws/src/rmcs_core/src/hardware/device/dm_motor.hpp` 现在不是“协议实现类”，它只是一个旧 `librmcs::device::DmMotor` 的外壳。  
但这个旧父类已经不存在了，所以你不能只改 `#include`，必须把 `DmMotor` 改成和现在仓库里的 `DjiMotor` / `LkMotor` 一样的“本地自管协议”的独立设备类。

最关键的证据就在这里：

- [dm_motor.hpp](/home/oiopi/projects/RMCS/rmcs_ws/src/rmcs_core/src/hardware/device/dm_motor.hpp:4) 还在 `#include "librmcs/device/dm_motor.hpp"`
- [dm_motor.hpp](/home/oiopi/projects/RMCS/rmcs_ws/src/rmcs_core/src/hardware/device/dm_motor.hpp:8) 还在 `class DmMotor : public librmcs::device::DmMotor`
- [dm_motor.hpp](/home/oiopi/projects/RMCS/rmcs_ws/src/rmcs_core/src/hardware/device/dm_motor.hpp:31) 到 [dm_motor.hpp](/home/oiopi/projects/RMCS/rmcs_ws/src/rmcs_core/src/hardware/device/dm_motor.hpp:99) 的核心逻辑几乎全部依赖旧父类

所以要改的不是“头文件路径”，而是“类结构”。

## 你应该把它改成什么结构

参考现在已经迁完的新风格：

- [dji_motor.hpp](/home/oiopi/projects/RMCS/rmcs_ws/src/rmcs_core/src/hardware/device/dji_motor.hpp:22)
- [lk_motor.hpp](/home/oiopi/projects/RMCS/rmcs_ws/src/rmcs_core/src/hardware/device/lk_motor.hpp:24)
- [can_packet.hpp](/home/oiopi/projects/RMCS/rmcs_ws/src/rmcs_core/src/hardware/device/can_packet.hpp:46)

`DmMotor` 应该从：

```cpp
class DmMotor : public librmcs::device::DmMotor {
    ...
};
```

改成：

```cpp
class DmMotor {
public:
    enum class Type : uint8_t { kDM8009 };

    struct Config {
        explicit Config(Type motor_type);
        Config& set_encoder_zero_point(int value);
        Config& set_reversed();
        Config& set_reduction_ratio(double value);
        Config& enable_multi_turn_angle();

        Type motor_type;
        int encoder_zero_point = 0;
        double reduction_ratio = 1.0;
        bool reversed = false;
        bool multi_turn_angle_enabled = false;
    };

    DmMotor(
        rmcs_executor::Component& status_component,
        rmcs_executor::Component& command_component,
        const std::string& name_prefix);

    DmMotor(
        rmcs_executor::Component& status_component,
        rmcs_executor::Component& command_component,
        const std::string& name_prefix,
        const Config& config);

    void configure(const Config& config);

    void store_status(std::span<const std::byte> can_data);
    void update_status();

    double angle() const;
    double velocity() const;
    double torque() const;
    double max_torque() const;
    uint8_t error_state() const;

    double control_torque() const;
    double control_angle() const;
    double control_velocity() const;

    int calibrate_zero_point();

    static CanPacket8 generate_enable_command();
    static CanPacket8 generate_clear_error_command();
    CanPacket8 generate_disable_command() const;
    CanPacket8 generate_torque_command(double control_torque) const;
    CanPacket8 generate_angle_command(double control_angle) const;
    CanPacket8 generate_command() const;

private:
    // 原始CAN反馈缓存
    std::atomic<CanPacket8> can_data_;

    // 配置与换算参数
    int encoder_zero_point_;
    int last_raw_angle_;
    bool reversed_;
    bool multi_turn_angle_enabled_;
    int64_t multi_turn_angle_;

    double angle_;
    double velocity_;
    double torque_;
    double max_torque_;
    uint8_t motor_state_;

    // component接口
    rmcs_executor::Component::OutputInterface<double> angle_output_;
    rmcs_executor::Component::OutputInterface<double> velocity_output_;
    rmcs_executor::Component::OutputInterface<double> torque_output_;
    rmcs_executor::Component::OutputInterface<double> max_torque_output_;

    rmcs_executor::Component::InputInterface<double> control_angle_;
    rmcs_executor::Component::InputInterface<double> control_velocity_;
    rmcs_executor::Component::InputInterface<double> control_torque_;
};
```

## 哪些旧内容必须删掉

下面这些都不能再保留“调用父类”的写法：

- `#include "librmcs/device/dm_motor.hpp"`  
  位置：[dm_motor.hpp](/home/oiopi/projects/RMCS/rmcs_ws/src/rmcs_core/src/hardware/device/dm_motor.hpp:4)
- `class DmMotor : public librmcs::device::DmMotor`  
  位置：[dm_motor.hpp](/home/oiopi/projects/RMCS/rmcs_ws/src/rmcs_core/src/hardware/device/dm_motor.hpp:8)
- `librmcs::device::DmMotor::configure(config)`  
  位置：[dm_motor.hpp](/home/oiopi/projects/RMCS/rmcs_ws/src/rmcs_core/src/hardware/device/dm_motor.hpp:31)
- `librmcs::device::DmMotor::update_status()`  
  位置：[dm_motor.hpp](/home/oiopi/projects/RMCS/rmcs_ws/src/rmcs_core/src/hardware/device/dm_motor.hpp:37)
- `error_state()`  
  位置：[dm_motor.hpp](/home/oiopi/projects/RMCS/rmcs_ws/src/rmcs_core/src/hardware/device/dm_motor.hpp:43)
- `generate_enable_command()` / `generate_clear_error_command()` / `generate_disable_command()` / `generate_torque_command()` / `generate_angle_command()` 的所有父类转发  
  位置：[dm_motor.hpp](/home/oiopi/projects/RMCS/rmcs_ws/src/rmcs_core/src/hardware/device/dm_motor.hpp:60)

这些全部都要变成本类自己实现。

## 哪些接口必须保留不变

`wheelleg-infantry-ai.cpp` 和控制器已经依赖了 `DmMotor` 的公开接口，新的 `DmMotor` 必须继续提供这些能力。

### 1. `Config`

四个髋关节电机都在这样配置：

- [wheelleg-infantry-ai.cpp](/home/oiopi/projects/RMCS/rmcs_ws/src/rmcs_core/src/hardware/wheelleg-infantry-ai.cpp:303)
- [wheelleg-infantry-ai.cpp](/home/oiopi/projects/RMCS/rmcs_ws/src/rmcs_core/src/hardware/wheelleg-infantry-ai.cpp:309)
- [wheelleg-infantry-ai.cpp](/home/oiopi/projects/RMCS/rmcs_ws/src/rmcs_core/src/hardware/wheelleg-infantry-ai.cpp:315)
- [wheelleg-infantry-ai.cpp](/home/oiopi/projects/RMCS/rmcs_ws/src/rmcs_core/src/hardware/wheelleg-infantry-ai.cpp:319)

所以新类至少要继续支持：

- `Type::DM8009` 或你统一改成 `Type::kDM8009`
- `.set_encoder_zero_point(...)`
- `.set_reversed()`

如果你把枚举命名改成新风格，`wheelleg-infantry-ai.cpp` 也要同步改。

### 2. `store_status(...)`

当前四个髋关节回包都走 `store_status(...)`：

- [wheelleg-infantry-ai.cpp](/home/oiopi/projects/RMCS/rmcs_ws/src/rmcs_core/src/hardware/wheelleg-infantry-ai.cpp:469)
- [wheelleg-infantry-ai.cpp](/home/oiopi/projects/RMCS/rmcs_ws/src/rmcs_core/src/hardware/wheelleg-infantry-ai.cpp:471)
- [wheelleg-infantry-ai.cpp](/home/oiopi/projects/RMCS/rmcs_ws/src/rmcs_core/src/hardware/wheelleg-infantry-ai.cpp:484)
- [wheelleg-infantry-ai.cpp](/home/oiopi/projects/RMCS/rmcs_ws/src/rmcs_core/src/hardware/wheelleg-infantry-ai.cpp:486)

建议把接口改成和 `DjiMotor`/`LkMotor` 一样：

```cpp
void store_status(std::span<const std::byte> can_data);
```

这样 `wheelleg-infantry-ai.cpp` 就不需要再先把 `data.can_data` 转成 `uint64_t`。

### 3. `update_status()`

四个髋关节在 update 阶段都会调用：

- [wheelleg-infantry-ai.cpp](/home/oiopi/projects/RMCS/rmcs_ws/src/rmcs_core/src/hardware/wheelleg-infantry-ai.cpp:381)

这个函数需要自己完成：

- 原始反馈包解析
- 编码器零点校正
- 多圈角度处理
- 速度换算
- 力矩换算
- `motor_state_`/`error_state` 更新
- component 输出赋值

### 4. `generate_command()`

四个髋关节发包都在直接用：

- [wheelleg-infantry-ai.cpp](/home/oiopi/projects/RMCS/rmcs_ws/src/rmcs_core/src/hardware/wheelleg-infantry-ai.cpp:405)
- [wheelleg-infantry-ai.cpp](/home/oiopi/projects/RMCS/rmcs_ws/src/rmcs_core/src/hardware/wheelleg-infantry-ai.cpp:411)
- [wheelleg-infantry-ai.cpp](/home/oiopi/projects/RMCS/rmcs_ws/src/rmcs_core/src/hardware/wheelleg-infantry-ai.cpp:428)
- [wheelleg-infantry-ai.cpp](/home/oiopi/projects/RMCS/rmcs_ws/src/rmcs_core/src/hardware/wheelleg-infantry-ai.cpp:434)

当前逻辑是：

- `motor_state_ == 0` 发 enable
- `motor_state_ == 1` 发 torque command
- 否则发 clear error

这个行为来自旧包装：

- [dm_motor.hpp](/home/oiopi/projects/RMCS/rmcs_ws/src/rmcs_core/src/hardware/device/dm_motor.hpp:76)
- [dm_motor.hpp](/home/oiopi/projects/RMCS/rmcs_ws/src/rmcs_core/src/hardware/device/dm_motor.hpp:86)

新实现如果不想改 wheelleg 主逻辑，最好先保持这个状态机行为。

### 5. `calibrate_zero_point()`

底盘髋关节校准回调直接依赖它：

- [wheelleg-infantry-ai.cpp](/home/oiopi/projects/RMCS/rmcs_ws/src/rmcs_core/src/hardware/wheelleg-infantry-ai.cpp:123)
- [wheelleg-infantry-ai.cpp](/home/oiopi/projects/RMCS/rmcs_ws/src/rmcs_core/src/hardware/wheelleg-infantry-ai.cpp:126)

所以新类必须继续提供 `calibrate_zero_point()`。

### 6. 组件 topic 接口

`wheel_leg_controller.cpp` 已经把髋关节输入输出主题写死了：

- 状态输入：[wheel_leg_controller.cpp](/home/oiopi/projects/RMCS/rmcs_ws/src/rmcs_core/src/controller/chassis/wheel_leg/wheel_leg_controller.cpp:50)
- 角度/速度/力矩输入：[wheel_leg_controller.cpp](/home/oiopi/projects/RMCS/rmcs_ws/src/rmcs_core/src/controller/chassis/wheel_leg/wheel_leg_controller.cpp:65)
- 控制输出：[wheel_leg_controller.cpp](/home/oiopi/projects/RMCS/rmcs_ws/src/rmcs_core/src/controller/chassis/wheel_leg/wheel_leg_controller.cpp:94)

所以新 `DmMotor` 里这些注册必须保留：

- `name_prefix + "/angle"`
- `name_prefix + "/velocity"`
- `name_prefix + "/torque"`
- `name_prefix + "/max_torque"`
- `name_prefix + "/control_torque"`
- `name_prefix + "/control_angle"`
- `name_prefix + "/control_velocity"`

## 最推荐的改法

### 第一步

先完全仿照 `DjiMotor` / `LkMotor` 的结构重写 `DmMotor`：

- 不再继承任何 `librmcs::device::*`
- 使用本地 `Config`
- 使用 `std::atomic<CanPacket8>` 缓存原始回包
- `store_status(std::span<const std::byte>)`
- `update_status()` 内自己做解析与输出
- `generate_*_command()` 内自己做协议编码

### 第二步

把 `wheelleg-infantry-ai.cpp` 里 DM 电机接口一并对齐到新风格：

- 把 `store_status(can_data_to_u64(data.can_data))` 改成 `store_status(data.can_data)`
- 把 `generate_command()` 返回类型从旧 `uint64_t` 改成 `CanPacket8`
- 发包时直接：

```cpp
builder.can1_transmit({
    .can_id = 0x01,
    .can_data = chassis_hip_motors[0].generate_command().as_bytes(),
});
```

这样下面这些临时兼容写法就可以删掉：

- 顶部 `can_data_to_u64(...)`
- 对 DM 电机命令的 `as_byte_span(...)` 包装

### 第三步

统一命名风格。

你现在 wheelleg 文件里还在使用旧 enum 名：

- `DjiMotor::Type::M3508`
- `DjiMotor::Type::M2006`
- `LkMotor::Type::MG4010E_I10`
- `LkMotor::Type::MG5010E_I10`

但当前实际定义已经改成：

- [dji_motor.hpp](/home/oiopi/projects/RMCS/rmcs_ws/src/rmcs_core/src/hardware/device/dji_motor.hpp:24)
- [lk_motor.hpp](/home/oiopi/projects/RMCS/rmcs_ws/src/rmcs_core/src/hardware/device/lk_motor.hpp:26)

也就是：

- `DjiMotor::Type::kM3508`
- `DjiMotor::Type::kM2006`
- `LkMotor::Type::kMG4010Ei10`
- `LkMotor::Type::kMG5010Ei10`

所以 `dm_motor.hpp` 修完后，`wheelleg-infantry-ai.cpp` 还要继续修这些名字，不然下一轮还会编不过。

## 现在不建议做的事

### 1. 不要只改 include

只把：

```cpp
#include "librmcs/device/dm_motor.hpp"
```

改成别的路径没有意义，因为类主体依旧全靠旧父类。

### 2. 不要继续保留 `uint64_t` 风格的旧接口

这个仓库的硬件层已经明显迁到 `span<byte>` / `CanPacket8` 风格。  
如果 `DmMotor` 继续保留纯 `uint64_t` 风格，它会成为 wheelleg 里最后一个旧接口孤岛。

### 3. 不要先改控制器 topic

当前 wheelleg 控制器对髋关节 topic 依赖很多，先把 `DmMotor` 设备层做成兼容替换更稳。  
topic 名字不是这次的第一优先级。

## 一个实际可执行的最小修改集合

如果你的目标是“先编过，再继续细调协议”，最小集合是：

1. 重写 `rmcs_ws/src/rmcs_core/src/hardware/device/dm_motor.hpp`
2. 让它不再依赖 `librmcs::device::DmMotor`
3. 保留 `configure()` / `update_status()` / `store_status()` / `generate_command()` / `calibrate_zero_point()` 这些公开接口
4. 把 `wheelleg-infantry-ai.cpp` 的 DM 收发改成 `std::span<const std::byte>` + `CanPacket8`
5. 同步修正 `wheelleg-infantry-ai.cpp` 里过时的 `DjiMotor::Type::*` / `LkMotor::Type::*` 名称

## 最后一句

这次 `DmMotor` 的“结构修改”本质上是：

**从“旧 librmcs 设备类的继承包装器”，改成“和 `DjiMotor` / `LkMotor` 一样的本地独立设备实现类”。**

如果你愿意，我下一步可以直接继续把这个 `dm_motor.hpp` 按这个结构改成可编译版本。
