# `PidController` vs `ErrorPidController`

## Short Answer

They both use the same PID core:

- `PidCalculator`

So your intuition is partly correct: both of them are PID controllers.

The real difference is **where the error comes from**.

- `PidController` computes the error by itself:
  `error = setpoint - measurement`
- `ErrorPidController` does not compute that subtraction:
  it assumes the incoming value is already the error

So the difference is not the PID math. The difference is the **input contract**.

## Code-Level Difference

In `pid_controller.cpp`, the update logic is:

```cpp
auto err = *setpoint_ - *measurement_;
*control_ = *feedforward_ + pid_calculator_.update(err);
```

This means `PidController` needs:

- `measurement`
- `setpoint`
- `control`

and optionally:

- `feedforward`

In `error_pid_controller.cpp`, the update logic is:

```cpp
auto err = *measurement_;
*control_ = *feedforward_ + pid_calculator_.update(err);
```

This means `ErrorPidController` needs:

- `measurement`
- `control`

and optionally:

- `feedforward`

But here the parameter name `measurement` is a little misleading.
In practice, it is really:

- `error`

## Same PID Core

Both classes build the same object:

```cpp
PidCalculator(get_parameter("kp").as_double(),
              get_parameter("ki").as_double(),
              get_parameter("kd").as_double())
```

Both also configure the same limits:

- `integral_min`
- `integral_max`
- `integral_split_min`
- `integral_split_max`
- `output_min`
- `output_max`

So these two plugins do **not** represent two different PID formulas.

They are two different wrappers around the same formula.

## When To Use Each One

### Use `PidController`

Use it when you have:

- a real measured value
- a desired target value

and you want the component to compute:

- `setpoint - measurement`

Typical examples:

- wheel velocity loop
- friction wheel velocity loop
- bullet feeder velocity loop

Example from this repo:

- `measurement: /gimbal/left_friction/velocity`
- `setpoint: /gimbal/left_friction/control_velocity`
- `control: /gimbal/left_friction/control_torque`

That is a standard PID usage:

```text
velocity_error = target_velocity - actual_velocity
```

### Use `ErrorPidController`

Use it when another component has already computed the error for you.

Typical examples:

- angle error produced by a gimbal controller
- wrapped angle error where simple subtraction is unsafe
- any preprocessed error signal

Example from this repo:

- `measurement: /gimbal/yaw/control_angle_error`
- `control: /gimbal/yaw/control_velocity`

Here the upstream component already outputs:

- `/gimbal/yaw/control_angle_error`

So this plugin should not subtract a second time.

Its job is just:

```text
control = PID(error)
```

## Why `ErrorPidController` Exists

At first glance it may look redundant, but it solves a real problem:

sometimes the error is not just:

```text
target - measurement
```

For example:

- angle error may need wrap-around handling
- the error may come from a higher-level controller
- the error may already include extra logic or filtering

If you already have a processed error signal, using `PidController` would force you to fake a setpoint/measurement pair again.

`ErrorPidController` avoids that.

## Repo Usage Pattern

This repository already uses them differently in YAML:

### `ErrorPidController`

Used for gimbal angle-related loops:

- `measurement: /gimbal/yaw/control_angle_error`
- `measurement: /gimbal/pitch/control_angle_error`

This strongly suggests the upstream gimbal module already computes angle error.

### `PidController`

Used for velocity loops:

- friction wheel velocity
- bullet feeder velocity
- chassis wheel velocity

This is the normal case where:

- actual velocity is measured directly
- target velocity is given separately

## Important Naming Caveat

The main source of confusion is that `ErrorPidController` still names its input parameter:

- `measurement`

But semantically it is really:

- `error`

So if you read the code literally, it can look like both plugins consume a measurement.
That is not true in behavior.

Their meanings are:

- `PidController.measurement` = actual measured process value
- `ErrorPidController.measurement` = already-computed control error

## `SmartInput` Detail

Both controllers use `SmartInput`, which means an input parameter can be:

- a topic/interface name
- a constant double
- or a negated input name like `-/some/value`

So both controllers are flexible in wiring, but that does not change the main distinction:

- one computes error internally
- one receives error externally

## Final Summary

If you compress the difference to one sentence:

- `PidController` is for `setpoint` + `measurement`
- `ErrorPidController` is for precomputed `error`

Both use the same `PidCalculator`.
They are different wrappers for different signal wiring patterns, not different PID algorithms.
