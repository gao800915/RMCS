# 新版 `dm_motor.hpp` 编写指南

这份指南的目标是：指导你基于下面几类文件，重新写出一个新的
`rmcs_ws/src/rmcs_core/src/hardware/device/dm_motor.hpp`。

你现在手上有四类关键材料：

- 当前 RMCS 里的旧包装器 `rmcs_core/src/hardware/device/dm_motor.hpp`
- 旧版 `librmcs/device/dm_motor.hpp` 原始实现
- 当前 RMCS 里的新风格 `dji_motor.hpp`
- 当前 RMCS 里的新风格 `lk_motor.hpp`

真正要做的事情不是“修一个 include 路径”，而是：

**把旧版 DM 电机协议实现，改写成当前 RMCS 3.0 风格的本地独立设备类。**

## 1. 先搞清楚这几个文件各自是什么角色

### 当前本地 `dm_motor.hpp`

它不是 DM 电机协议本体，它只是一个壳：

- 负责注册 RMCS component 输入输出
- 继承 `librmcs::device::DmMotor`
- 绝大多数真实逻辑都交给父类

所以它现在会失败，是因为父类已经没了。

### 旧版原始 `librmcs::device::DmMotor`

这个文件才是 DM 电机的真实协议来源。里面包含：

- `DM8009` 的配置常量
- 反馈帧解析方式
- angle / velocity / torque 的换算方式
- enable / clear-error / MIT 控制帧的打包方式

你要迁移的“协议逻辑”基本都来自这里。

### 当前 `DjiMotor` 和 `LkMotor`

这两个文件不是给你 DM 协议用的，而是给你“新架构模板”用的。它们告诉你现在 RMCS 硬件层应该怎么写：

- 不再依赖 `librmcs::device::*`
- 自己持有原始 CAN 数据缓存
- `store_status(std::span<const std::byte>)`
- `update_status()` 自己做解析
- command 生成函数返回本地包对象
- component 输入输出在类内直接注册

所以：

- 协议细节看旧 `librmcs::device::DmMotor`
- 新类结构看 `DjiMotor` / `LkMotor`

## 2. 先确定最终目标结构

不要再写成这种旧结构：

```cpp
class DmMotor : public librmcs::device::DmMotor {
    ...
};
```

要改成这种新结构：

```cpp
namespace rmcs_core::hardware::device {

class DmMotor {
public:
    enum class Type : uint8_t { kDM8009 };

    struct Config { ... };

    DmMotor(...);
    DmMotor(..., const Config& config);

    void configure(const Config& config);
    void store_status(std::span<const std::byte> can_data);
    void update_status();

    int64_t calibrate_zero_point();
    uint8_t error_state() const;

    double angle() const;
    double velocity() const;
    double torque() const;
    double max_torque() const;
    double position() const;
    double mos_temperature() const;
    double rotor_temperature() const;

    double control_torque() const;
    double control_angle() const;
    double control_velocity() const;

    static CanPacket8 generate_enable_command();
    static CanPacket8 generate_clear_error_command();
    CanPacket8 generate_disable_command() const;
    CanPacket8 generate_torque_command(double control_torque) const;
    CanPacket8 generate_angle_command(
        double control_angle, double control_kp = 60.0, double control_kd = 2.0) const;
    CanPacket8 generate_command() const;

private:
    ...
};

}
```

这就是你该追求的目标形态。

## 3. 第一步：先改文件头和 include

把旧的：

```cpp
#include "librmcs/device/dm_motor.hpp"
```

直接删掉。

然后把文件头改成和当前 `DjiMotor` / `LkMotor` 类似的依赖：

```cpp
#pragma once

#include <algorithm>
#include <atomic>
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <numbers>
#include <span>
#include <string>
#include <utility>

#include <rmcs_executor/component.hpp>

#include "hardware/device/can_packet.hpp"
```

如果你后面会用到：

- `std::clamp`
- `std::isnan`
- `std::bit_cast`
- `std::span`

这一组 include 就足够了。

## 4. 第二步：先写 `Type` 和 `Config`

旧版原始实现里只有一种电机类型：

```cpp
enum class Type : uint8_t { DM8009 };
```

为了和现在仓库命名风格一致，建议改成：

```cpp
enum class Type : uint8_t { kDM8009 };
```

然后写新的 `Config`：

```cpp
struct Config {
    explicit Config(Type motor_type)
        : motor_type(motor_type) {}

    Config& set_encoder_zero_point(int value) { return encoder_zero_point = value, *this; }
    Config& set_reversed() { return reversed = true, *this; }
    Config& enable_multi_turn_angle() { return multi_turn_angle_enabled = true, *this; }

    Type motor_type;
    int encoder_zero_point = 0;
    bool reversed = false;
    bool multi_turn_angle_enabled = false;
};
```

现在不要乱加别的字段。先把最小兼容集合做好。

## 5. 第三步：先把构造函数写出来

这一部分基本是机械照搬 `DjiMotor` / `LkMotor` 的写法。

### 必须注册的输出

- `name_prefix + "/angle"`
- `name_prefix + "/velocity"`
- `name_prefix + "/torque"`
- `name_prefix + "/max_torque"`

### 必须注册的输入

- `name_prefix + "/control_angle"`
- `name_prefix + "/control_velocity"`
- `name_prefix + "/control_torque"`

这些不能省，因为 wheelleg 控制器已经依赖这些 topic。

建议先写成：

```cpp
DmMotor(
    rmcs_executor::Component& status_component,
    rmcs_executor::Component& command_component,
    const std::string& name_prefix)
    : angle_(0.0)
    , velocity_(0.0)
    , torque_(0.0) {
    status_component.register_output(name_prefix + "/angle", angle_output_, 0.0);
    status_component.register_output(name_prefix + "/velocity", velocity_output_, 0.0);
    status_component.register_output(name_prefix + "/torque", torque_output_, 0.0);
    status_component.register_output(name_prefix + "/max_torque", max_torque_output_, 0.0);

    command_component.register_input(name_prefix + "/control_angle", control_angle_, false);
    command_component.register_input(name_prefix + "/control_velocity", control_velocity_, false);
    command_component.register_input(name_prefix + "/control_torque", control_torque_, false);
}
```

然后加上带 `Config` 的委托构造：

```cpp
DmMotor(..., const Config& config)
    : DmMotor(status_component, command_component, name_prefix) {
    configure(config);
}
```

## 6. 第四步：把旧版常量搬进 `configure()`

这是你开始迁移真实协议逻辑的第一步。

旧版原始 `librmcs::device::DmMotor` 里，`DM8009` 的关键常量是：

- `raw_angle_max_ = 65535`
- `reduction_ratio = 1.0`
- `max_velocity_ = 45.0`
- `max_torque_ = 54.0`
- `max_angle_ = std::numbers::pi`
- `max_kp_ = 500.0`
- `max_kd_ = 5.0`

建议把 `configure()` 写成这样：

```cpp
void configure(const Config& config) {
    multi_turn_encoder_count_ = 0;
    last_raw_angle_ = 0;
    motor_state_ = 0;

    double reduction_ratio;
    switch (config.motor_type) {
    case Type::kDM8009:
        raw_angle_max_ = 65535;
        reduction_ratio = 1.0;
        max_velocity_ = 45.0;
        max_torque_ = 54.0;
        max_angle_ = std::numbers::pi;
        max_kp_ = 500.0;
        max_kd_ = 5.0;
        break;
    default:
        std::unreachable();
    }

    encoder_zero_point_ = config.encoder_zero_point % raw_angle_max_;
    if (encoder_zero_point_ < 0)
        encoder_zero_point_ += raw_angle_max_;

    multi_turn_angle_enabled_ = config.multi_turn_angle_enabled;

    const double sign = config.reversed ? -1.0 : 1.0;

    status_angle_to_angle_coefficient_ = sign / raw_angle_max_ * 2 * std::numbers::pi;
    angle_to_command_angle_coefficient_ = sign * reduction_ratio * kRadToDeg;

    status_velocity_to_velocity_coefficient_ = sign / reduction_ratio;
    velocity_to_command_velocity_coefficient_ = sign * reduction_ratio;

    status_torque_to_torque_coefficient_ = sign * reduction_ratio;
    torque_to_command_torque_coefficient_ = 1.0 / status_torque_to_torque_coefficient_;

    *max_torque_output_ = max_torque();
}
```

这里有一个原则：

- 第一版先完全保留旧常量
- 不要一边迁移一边“优化参数”
- 先保证行为一致，再考虑改进

## 7. 第五步：把原始 CAN 缓存从 `uint64_t` 改成 `CanPacket8`

旧原始实现用的是：

```cpp
std::atomic<uint64_t> can_data_;
```

但当前 RMCS 3.0 的风格应该改成：

```cpp
std::atomic<CanPacket8> can_data_;
```

然后把 `store_status()` 写成：

```cpp
void store_status(std::span<const std::byte> can_data) {
    if (can_data.size() != 8) [[unlikely]]
        return;

    can_data_.store(CanPacket8{can_data}, std::memory_order_relaxed);
}
```

这样 `DmMotor` 就和 `DjiMotor` / `LkMotor` 保持一致了。

## 8. 第六步：按旧协议重写 `update_status()`

这是最关键的一步。

你现在已经有旧协议逻辑，所以不要重新设计。你只需要把它改写成新本地类风格。

### 旧 DM 回包布局

旧版原始实现里，反馈帧布局是：

- byte 0: id + motor_state
- byte 1~2: encoder
- byte 3~4 的高位部分: velocity
- byte 4 低 4 位 + byte 5: torque
- byte 6: MOS 温度
- byte 7: rotor 温度

所以你可以定义一个本地反馈结构：

```cpp
struct alignas(uint64_t) DmMotorFeedback {
    uint8_t id_state;
    uint8_t encoder_high;
    uint8_t encoder_low;
    uint8_t velocity_high;
    uint8_t velocity_low_torque_high;
    uint8_t torque_low;
    uint8_t mos_temperature;
    uint8_t rotor_temperature;
};
```

### 然后写 `update_status()`

建议基本按旧实现逐句搬运：

```cpp
void update_status() {
    const auto feedback =
        std::bit_cast<DmMotorFeedback>(can_data_.load(std::memory_order_relaxed));

    id_ = feedback.id_state & 0x0F;
    motor_state_ = feedback.id_state >> 4;

    const uint16_t encoder =
        (static_cast<uint16_t>(feedback.encoder_high) << 8) | feedback.encoder_low;
    const uint16_t velocity =
        (static_cast<uint16_t>(feedback.velocity_high) << 4)
        | (feedback.velocity_low_torque_high >> 4);
    const uint16_t torque =
        (static_cast<uint16_t>(feedback.velocity_low_torque_high & 0x0F) << 8)
        | feedback.torque_low;

    mos_temperature_ = static_cast<double>(feedback.mos_temperature);
    rotor_temperature_ = static_cast<double>(feedback.rotor_temperature);

    const int raw_angle = static_cast<int>(encoder);
    position_ = uint_to_double(encoder, -max_angle_, max_angle_, 16);

    int calibrated_raw_angle = encoder - encoder_zero_point_;
    if (calibrated_raw_angle < 0)
        calibrated_raw_angle += raw_angle_max_;

    if (!multi_turn_angle_enabled_) {
        angle_ = status_angle_to_angle_coefficient_ * static_cast<double>(calibrated_raw_angle);
        if (angle_ < 0)
            angle_ += 2 * std::numbers::pi;
    } else {
        auto diff = (calibrated_raw_angle - multi_turn_encoder_count_) & (raw_angle_max_ - 1);
        if (diff > (raw_angle_max_ >> 1))
            diff -= raw_angle_max_;

        multi_turn_encoder_count_ += diff;
        angle_ = status_angle_to_angle_coefficient_
               * static_cast<double>(multi_turn_encoder_count_);
    }

    last_raw_angle_ = raw_angle;

    velocity_ = status_velocity_to_velocity_coefficient_
              * uint_to_double(velocity, -max_velocity_, max_velocity_, 12);
    torque_ = status_torque_to_torque_coefficient_
            * uint_to_double(torque, -max_torque_, max_torque_, 12);

    *angle_output_ = angle();
    *velocity_output_ = velocity();
    *torque_output_ = torque();
}
```

这里本质上就是：

- 解析布局完全沿用旧版
- 组织形式改成新版本地类

## 9. 第七步：保留旧版数值转换函数

这两个函数不要乱改，直接沿用旧原始实现：

```cpp
static double uint_to_double(int x_int, double x_min, double x_max, int bits) {
    const double span = x_max - x_min;
    const double offset = x_min;
    return static_cast<double>(x_int) * span / static_cast<double>((1 << bits) - 1) + offset;
}

static int double_to_uint(double x_double, double x_min, double x_max, int bits) {
    const double span = x_max - x_min;
    const double offset = x_min;
    return static_cast<int>((x_double - offset) * static_cast<double>((1 << bits) - 1) / span);
}
```

这是 DM MIT 协议编码/解码的基础。

## 10. 第八步：补全控制输入访问函数

写法建议和新电机风格一致，但语义保留旧 DM 包装器的行为。

```cpp
double control_torque() const {
    if (control_torque_.ready()) [[likely]]
        return *control_torque_;
    else
        return std::numeric_limits<double>::quiet_NaN();
}

double control_angle() const {
    if (control_angle_.ready()) [[likely]]
        return *control_angle_;
    else
        return std::numeric_limits<double>::quiet_NaN();
}

double control_velocity() const {
    if (control_velocity_.ready()) [[likely]]
        return *control_velocity_;
    else
        return std::numeric_limits<double>::quiet_NaN();
}
```

注意这里建议返回 `NaN`，不要返回 `0.0`。因为旧的本地包装器本来就是这么做的。

## 11. 第九步：重建 command 生成函数，但返回 `CanPacket8`

旧原始实现返回的是 `uint64_t`。新版不要继续这样做。

新版应该返回：

```cpp
CanPacket8
```

### 11.1 `generate_enable_command()`

旧版就是固定的 8 字节：

```cpp
constexpr static CanPacket8 generate_enable_command() {
    const struct [[gnu::packed]] {
        uint8_t message[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
    } command alignas(CanPacket8){};
    return std::bit_cast<CanPacket8>(command);
}
```

### 11.2 `generate_clear_error_command()`

同样是固定 8 字节：

```cpp
constexpr static CanPacket8 generate_clear_error_command() {
    const struct [[gnu::packed]] {
        uint8_t message[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFB};
    } command alignas(CanPacket8){};
    return std::bit_cast<CanPacket8>(command);
}
```

### 11.3 写一个私有的 `generate_mit_command()`

建议单独写：

```cpp
CanPacket8 generate_mit_command(
    double control_angle,
    double control_velocity,
    double control_torque,
    double control_kp,
    double control_kd) const
```

在里面做四件事：

1. clamp angle / velocity / torque
2. 用 `double_to_uint` 转成协议整数
3. 按旧版 MIT 帧布局打 8 个字节
4. 返回 `std::bit_cast<CanPacket8>(command)`

建议实现骨架：

```cpp
CanPacket8 generate_mit_command(
    double control_angle,
    double control_velocity,
    double control_torque,
    double control_kp,
    double control_kd) const {
    control_angle = std::clamp(control_angle, -max_angle_, max_angle_);
    control_velocity = std::clamp(control_velocity, -max_velocity_, max_velocity_);
    control_torque = std::clamp(control_torque, -max_torque_, max_torque_);

    const uint16_t angle = double_to_uint(control_angle, -max_angle_, max_angle_, 16);
    const uint16_t velocity = double_to_uint(control_velocity, -max_velocity_, max_velocity_, 12);
    const uint16_t torque = double_to_uint(control_torque, -max_torque_, max_torque_, 12);
    const uint16_t kp = double_to_uint(control_kp, 0.0, max_kp_, 12);
    const uint16_t kd = double_to_uint(control_kd, 0.0, max_kd_, 12);

    const struct [[gnu::packed]] {
        uint8_t byte0;
        uint8_t byte1;
        uint8_t byte2;
        uint8_t byte3;
        uint8_t byte4;
        uint8_t byte5;
        uint8_t byte6;
        uint8_t byte7;
    } command alignas(CanPacket8){
        .byte0 = static_cast<uint8_t>(angle >> 8),
        .byte1 = static_cast<uint8_t>(angle),
        .byte2 = static_cast<uint8_t>(velocity >> 4),
        .byte3 = static_cast<uint8_t>(((velocity & 0x0F) << 4) | (kp >> 8)),
        .byte4 = static_cast<uint8_t>(kp),
        .byte5 = static_cast<uint8_t>(kd >> 4),
        .byte6 = static_cast<uint8_t>(((kd & 0x0F) << 4) | (torque >> 8)),
        .byte7 = static_cast<uint8_t>(torque),
    };

    return std::bit_cast<CanPacket8>(command);
}
```

### 11.4 `generate_disable_command()`

旧逻辑是“发一个 0 torque 的 MIT 指令”，不是单独的 disable 特殊帧：

```cpp
CanPacket8 generate_disable_command() const {
    return generate_mit_command(0.0, 0.0, 0.0, 0.0, 0.0);
}
```

### 11.5 `generate_torque_command()`

```cpp
CanPacket8 generate_torque_command(double control_torque) const {
    if (std::isnan(control_torque))
        return generate_disable_command();

    return generate_mit_command(0.0, 0.0, to_command_torque(control_torque), 0.0, 0.0);
}
```

### 11.6 `generate_angle_command()`

```cpp
CanPacket8 generate_angle_command(
    double control_angle,
    double control_kp = 60.0,
    double control_kd = 2.0) const {
    if (std::isnan(control_angle))
        return generate_disable_command();

    return generate_mit_command(control_angle, 0.0, 0.0, control_kp, control_kd);
}
```

## 12. 第十步：保留当前本地包装器的状态机策略

这一点很重要。

虽然旧原始 `librmcs::device::DmMotor` 只负责协议，但当前 RMCS 本地包装器额外加了一层策略：

- `motor_state_ == 0` 时发 enable
- `motor_state_ == 1` 时发 torque command
- 其他状态发 clear error

wheelleg 现在的行为默认依赖这个策略，所以第一版先保留：

```cpp
CanPacket8 generate_command() const {
    if (motor_state_ == 0) {
        return generate_enable_command();
    } else if (motor_state_ == 1) {
        return generate_torque_command(control_torque());
    } else {
        return generate_clear_error_command();
    }
}
```

如果你以后要恢复 angle 模式切换，也应该放在迁移成功之后再做。

## 13. 第十一步：保留这些 getter 和辅助函数

下面这些接口建议继续保留：

```cpp
int64_t calibrate_zero_point();
uint8_t error_state() const;

double angle() const;
double velocity() const;
double torque() const;
double max_torque() const;
double position() const;
double mos_temperature() const;
double rotor_temperature() const;
```

同时保留私有转换辅助：

```cpp
double to_command_torque(double torque) const;
double to_command_velocity(double velocity) const;
```

即使 `to_command_velocity()` 第一版没有直接用上，也可以保留。因为旧原始实现里本来就有它，以后如果扩展 DM 控制模式可能还会用到。

## 14. 第十二步：最后再补 private 成员

等方法都写完，再整理成员，最稳。

建议成员集合如下：

```cpp
std::atomic<CanPacket8> can_data_{CanPacket8{0}};

static constexpr double kDegToRad = std::numbers::pi / 180.0;
static constexpr double kRadToDeg = 180.0 / std::numbers::pi;

int raw_angle_max_{};
int encoder_zero_point_{};

bool multi_turn_angle_enabled_{false};
int64_t multi_turn_encoder_count_{0};
int last_raw_angle_{0};

double status_angle_to_angle_coefficient_{};
double angle_to_command_angle_coefficient_{};
double status_velocity_to_velocity_coefficient_{};
double velocity_to_command_velocity_coefficient_{};
double status_torque_to_torque_coefficient_{};
double torque_to_command_torque_coefficient_{};

uint8_t id_{0};
uint8_t motor_state_{0};

double angle_{0.0};
double velocity_{0.0};
double torque_{0.0};
double position_{0.0};

double max_angle_{0.0};
double max_torque_{0.0};
double max_velocity_{0.0};
double max_kp_{0.0};
double max_kd_{0.0};

double mos_temperature_{0.0};
double rotor_temperature_{0.0};

rmcs_executor::Component::OutputInterface<double> angle_output_;
rmcs_executor::Component::OutputInterface<double> velocity_output_;
rmcs_executor::Component::OutputInterface<double> torque_output_;
rmcs_executor::Component::OutputInterface<double> max_torque_output_;

rmcs_executor::Component::InputInterface<double> control_angle_;
rmcs_executor::Component::InputInterface<double> control_velocity_;
rmcs_executor::Component::InputInterface<double> control_torque_;
```

注意这里：

- 不需要继续保留当前破损包装器里的 `angle_multi_turn_`
- 旧原始协议真正使用的是 `multi_turn_encoder_count_`

## 15. 第十三步：新 `dm_motor.hpp` 写完后，`wheelleg-infantry-ai.cpp` 也必须一起改

这一步很重要。只改头文件是不够的。

### 15.1 改枚举名

如果你把枚举改成新风格：

```cpp
Type::kDM8009
```

那么 `wheelleg-infantry-ai.cpp` 里这些地方也要同步改：

```cpp
device::DmMotor::Config{device::DmMotor::Type::DM8009}
```

改成：

```cpp
device::DmMotor::Config{device::DmMotor::Type::kDM8009}
```

### 15.2 改接收路径

当前旧写法：

```cpp
const auto can_data = can_data_to_u64(data.can_data);
chassis_hip_motors[i].store_status(can_data);
```

新写法应该变成：

```cpp
chassis_hip_motors[i].store_status(data.can_data);
```

因为新的 `DmMotor` 应该直接接受 `std::span<const std::byte>`。

### 15.3 改发送路径

当前旧写法默认 `generate_command()` 返回 `uint64_t`：

```cpp
const auto left_front_hip_command = chassis_hip_motors[0].generate_command();
builder.can1_transmit({
    .can_id = 0x01,
    .can_data = as_byte_span(left_front_hip_command),
});
```

新的写法应该改成：

```cpp
builder.can1_transmit({
    .can_id = 0x01,
    .can_data = chassis_hip_motors[0].generate_command().as_bytes(),
});
```

这样才和当前新硬件层风格一致。

### 15.4 清理过时兼容胶水

当 DM 电机完全迁好之后，下面这些临时兼容函数对于 DM 路径就不再需要了：

- `can_data_to_u64(...)`
- 专门给 DM command 用的 `as_byte_span(...)`

别的旧设备如果还依赖它们，可以暂时保留；但 DM 不该再依赖这些旧转换。

## 16. 最安全的实际编码顺序

建议严格按下面顺序做：

1. 去掉对 `librmcs::device::DmMotor` 的继承
2. 换成本地 include 和 `CanPacket8`
3. 写 `Type`、`Config`、构造函数、topic 注册
4. 从旧原始实现迁移 `configure()` 中的常量和换算系数
5. 迁移 `store_status()` 和 `update_status()`
6. 迁移 `uint_to_double()` 与 `double_to_uint()`
7. 迁移 fixed command：enable 与 clear-error
8. 迁移 `generate_mit_command()`
9. 迁移 `generate_disable_command()`、`generate_torque_command()`、`generate_angle_command()`
10. 补回当前本地包装器的 `generate_command()` 状态机策略
11. 修改 `wheelleg-infantry-ai.cpp`，让 DM 改用 `span<byte>` 和 `CanPacket8`
12. 最后再修 `wheelleg-infantry-ai.cpp` 里其他无关但仍旧过时的 `DjiMotor` / `LkMotor` 枚举名

这样做的好处是：

- 先把协议层改对
- 再接硬件集成层
- 不会把问题混在一起

## 17. 第一版迁移时不要动的东西

第一版迁移时，下面这些都先不要改：

- `DM8009` 的常量
- MIT 帧的 bit 打包方式
- `generate_command()` 的状态机策略
- wheelleg 控制器 topic 名字
- hip CAN ID `0x01` 到 `0x04`

第一目标不是“优化”，而是：

**在新架构下先复现旧行为。**

## 18. 最后的理解方式

你可以把这次工作理解成三层拼装：

### 协议行为来源

来自旧原始 `librmcs::device::DmMotor`

- 常量
- 反馈解析
- MIT 打包
- enable / clear-error

### 架构风格来源

来自当前 `DjiMotor` / `LkMotor`

- 本地类
- packet 缓存
- `store_status(span<byte>)`
- `update_status()`
- `CanPacket8`
- component 输入输出注册

### 外部接口兼容要求

来自当前本地 `dm_motor.hpp` 和 wheelleg 代码

- 必须保留的 public API
- 必须保留的 topic
- 当前 wheelleg 的状态机行为

所以真正正确的做法就是：

**把旧 DM 协议逻辑，从旧 `librmcs` 里拆出来，按当前 `DjiMotor` / `LkMotor` 的本地设备风格重写出来。**

这就是新的 `dm_motor.hpp` 应该怎么写。

---

## 补充检查意见：你当前这版 `dm_motor.hpp` 下一步还要做什么

下面这部分是针对你已经改过的 `rmcs_ws/src/rmcs_core/src/hardware/device/dm_motor.hpp` 的补充检查意见。

我先给结论：

- 你现在的方向是对的，已经从“继承旧 `librmcs::device::DmMotor`”走到了“本地独立类”的方向。
- 但这份文件目前还没有收口，里面同时存在“语法级问题”、“类内未完成定义”和“外部集成端还没联动修改”三类问题。
- 所以你下一步不要急着继续细调协议，先把这三类问题按顺序清掉。

### 一、先修最基础的结构错误

这是第一优先级，因为这些问题会直接导致文件本身就不能作为一个完整类定义存在。

#### 1. 类和命名空间没有正确闭合

你文件末尾现在停在：

```cpp
rmcs_executor::Component::InputInterface<double> control_torque_;
// namespace rmcs_core::hardware::device
```

这里明显少了：

```cpp
};

} // namespace rmcs_core::hardware::device
```

也就是说：

- 类 `DmMotor` 没有 `};`
- 命名空间也没有真正闭合

这个必须先补上。

#### 2. 第二个构造函数现在是非法写法

你现在写的是：

```cpp
DmMotor(..., const Config& config)
    : DmMotor(status_component, command_component, name_prefix) {
    configure(config);
}
```

这在 C++ 里不是一个合法可编译的真实构造函数定义。问题有两个：

1. `...` 这里只是占位思路，不是实际参数列表
2. `status_component`、`command_component`、`name_prefix` 在这个作用域里根本不存在

你必须改成完整真实签名，例如：

```cpp
DmMotor(
    rmcs_executor::Component& status_component,
    rmcs_executor::Component& command_component,
    const std::string& name_prefix,
    const Config& config)
    : DmMotor(status_component, command_component, name_prefix) {
    configure(config);
}
```

这个问题是确定要修的，不是风格建议。

### 二、把“只声明没定义”的函数全部补齐

这是第二优先级。

你当前文件里已经有一批函数被使用了，但只写了声明，没有给出定义。  
这会导致即使头文件语法过了，后面链接或模板实例化时也会出问题。

你现在还没补完的函数包括：

```cpp
int64_t calibrate_zero_point();
uint8_t error_state() const;

double angle() const;
double velocity() const;
double torque() const;
double max_torque() const;
double position() const;
double mos_temperature() const;
double rotor_temperature() const;
```

这些建议直接写成类内短函数，和 `DjiMotor` / `LkMotor` 风格保持一致，例如：

```cpp
int64_t calibrate_zero_point() {
    multi_turn_encoder_count_ = 0;
    encoder_zero_point_ = last_raw_angle_;
    return encoder_zero_point_;
}

uint8_t error_state() const { return motor_state_; }

double angle() const { return angle_; }
double velocity() const { return velocity_; }
double torque() const { return torque_; }
double max_torque() const { return max_torque_; }
double position() const { return position_; }
double mos_temperature() const { return mos_temperature_; }
double rotor_temperature() const { return rotor_temperature_; }
```

### 三、你还缺少几个真正被调用到的私有辅助定义

这是第三优先级。

你当前代码已经调用了这些内容：

- `DmMotorFeedback`
- `uint_to_double(...)`
- `double_to_uint(...)`
- `to_command_torque(...)`
- `to_command_velocity(...)`

但从你现在文件内容看，它们还没有完整定义。

#### 1. `DmMotorFeedback` 结构体还没写出来

你在 `update_status()` 里用了：

```cpp
std::bit_cast<DmMotorFeedback>(...)
```

但类里没有看到这个结构体定义。

你必须补一个本地反馈结构，例如：

```cpp
struct alignas(uint64_t) DmMotorFeedback {
    uint8_t id_state;
    uint8_t encoder_high;
    uint8_t encoder_low;
    uint8_t velocity_high;
    uint8_t velocity_low_torque_high;
    uint8_t torque_low;
    uint8_t mos_temperature;
    uint8_t rotor_temperature;
};
```

#### 2. `uint_to_double()` 和 `double_to_uint()` 还没补

你在：

- `position_ = uint_to_double(...)`
- `velocity_ = ... * uint_to_double(...)`
- `torque_ = ... * uint_to_double(...)`
- `generate_mit_command()` 里又用了 `double_to_uint(...)`

但当前文件里还没有这两个函数定义。

建议直接照旧版原始 `librmcs::device::DmMotor` 补回去。

#### 3. `to_command_torque()` 至少必须补

你在：

```cpp
return generate_mit_command(0.0, 0.0, to_command_torque(control_torque), 0.0, 0.0);
```

已经调用了 `to_command_torque()`，但这个函数目前还没看到定义。

建议补成：

```cpp
double to_command_torque(double torque) const {
    return torque_to_command_torque_coefficient_ * torque;
}
```

`to_command_velocity()` 目前虽然未直接参与 `generate_command()` 主路径，但也建议一起补上：

```cpp
double to_command_velocity(double velocity) const {
    return velocity_to_command_velocity_coefficient_ * velocity;
}
```

### 四、你现在的文件虽然“像”新风格，但还不够像 `DjiMotor` / `LkMotor`

这部分不是立即导致编不过的问题，但会影响后面维护。

#### 1. 成员和方法顺序还比较乱

你现在的排列顺序大概是：

- 一些 public 方法
- 一些声明
- `generate_mit_command()`
- private 成员

建议你整理成更清楚的结构：

1. `Type`
2. `Config`
3. 构造函数
4. `configure()`
5. `store_status()`
6. `update_status()`
7. control getter
8. command generator
9. status getter / calibrate
10. private helper
11. private struct
12. private members

这样和 `DjiMotor` / `LkMotor` 一致，后续你自己看也会轻松很多。

#### 2. 缩进和排版也建议一次性整理

你这版里已经出现了明显的排版失衡，例如：

- 构造函数缩进不统一
- `control_torque()` 体内没有按类内风格缩进
- `generate_enable_command()` / `generate_clear_error_command()` 缩进不统一
- `private:` 下的成员整体左移

这不影响协议逻辑，但会严重影响你自己后面排查问题。

建议你在结构修完后，统一按 `DjiMotor` 的排版风格重排一次。

### 五、wheelleg 集成端现在还没有跟着你的新 `DmMotor` 一起改

这是第四优先级，但它一定要做。

就算你把 `dm_motor.hpp` 自己修完整了，`wheelleg-infantry-ai.cpp` 现在仍然还是旧接口用法。

#### 1. `Type::DM8009` 还没改成新枚举名

你新的 `DmMotor` 里已经是：

```cpp
Type::kDM8009
```

但 `wheelleg-infantry-ai.cpp` 里还是：

```cpp
device::DmMotor::Type::DM8009
```

这几处都要改成：

```cpp
device::DmMotor::Type::kDM8009
```

#### 2. 接收路径还在走旧 `uint64_t` 适配

你新的 `store_status()` 已经是：

```cpp
void store_status(std::span<const std::byte> can_data)
```

但 `wheelleg-infantry-ai.cpp` 还在这样写：

```cpp
const auto can_data = can_data_to_u64(data.can_data);
chassis_hip_motors[i].store_status(can_data);
```

对于 DM 电机，应该改成：

```cpp
chassis_hip_motors[i].store_status(data.can_data);
```

#### 3. 发送路径还在把 DM command 当成旧 `uint64_t`

你现在新的 `generate_command()` 返回的是 `CanPacket8`。  
但 wheelleg 还在这样发：

```cpp
const auto left_front_hip_command = chassis_hip_motors[0].generate_command();
builder.can1_transmit({
    .can_id = 0x01,
    .can_data = as_byte_span(left_front_hip_command),
});
```

新写法应该改成：

```cpp
builder.can1_transmit({
    .can_id = 0x01,
    .can_data = chassis_hip_motors[0].generate_command().as_bytes(),
});
```

也就是说，DM 电机这一条链路要完全切换到：

- `std::span<const std::byte>` 接收
- `CanPacket8` 发送

### 六、还有一个现实问题：你当前 wheelleg 文件里别的电机枚举名也还是旧的

这不是 `dm_motor.hpp` 本身的问题，但你修完 DM 之后，下一轮编译大概率还会继续卡在这里：

- `device::LkMotor::Type::MG4010E_I10`
- `device::DjiMotor::Type::M2006`
- `device::DjiMotor::Type::M3508`

而当前真实定义已经是新名字：

- `LkMotor::Type::kMG4010Ei10`
- `DjiMotor::Type::kM2006`
- `DjiMotor::Type::kM3508`

所以你要有心理准备：

- 修完 `dm_motor.hpp` 不是终点
- 只是把第一个大障碍拆掉
- 下一步还得继续修 `wheelleg-infantry-ai.cpp` 里的新旧接口混用问题

### 七、最推荐的下一步执行顺序

如果你想最省时间，我建议按这个顺序继续改：

1. 先把 `dm_motor.hpp` 自己补成一个完整闭合的类
2. 把所有“只声明没定义”的函数补齐
3. 把 `DmMotorFeedback`、`uint_to_double()`、`double_to_uint()`、`to_command_torque()` 等私有辅助补齐
4. 整理缩进和类内顺序
5. 再去改 `wheelleg-infantry-ai.cpp`，把 DM 的收发接口改成 `span<byte>` + `CanPacket8`
6. 最后再修 `DjiMotor` / `LkMotor` 的旧枚举名

### 八、简短判断

如果只评价你现在这个文件：

- 架构方向：对
- 协议迁移思路：对
- 当前完成度：还没有到“可接入编译”的程度

你现在离“能继续往前编”还差的不是协议理论，而是这些基本收尾工作：

- 把类定义补完整
- 把未实现函数补完整
- 把 wheelleg 调用端同步改完整

先把这三件事做完，再看下一轮编译报错，才是最有效率的做法。
