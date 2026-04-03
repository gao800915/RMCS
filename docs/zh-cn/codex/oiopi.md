# OIOPI Two-M3508 Review

## Conclusion

As of 2026-04-01, the current code does not yet achieve the goal of:

- driving two `M3508` motors, and
- getting their status in a usable way.

The structure is close, because the OIOPI hardware file already creates two wheel motors:

- `/chassis/left_wheel`
- `/chassis/right_wheel`

and it already contains CAN send/receive logic for them.

However, there are still several hard blockers.

## Files Checked

I checked these three files:

- `rmcs_ws/src/rmcs_core/src/hardware/oiopi_wheelleg_infantry.cpp`
- `rmcs_ws/src/rmcs_bringup/config/oiopi-wheelleg-infantry.yaml`
- `rmcs_ws/src/rmcs_core/plugins.xml`

I also compared them with the existing reference implementation:

- `rmcs_ws/src/rmcs_core/src/hardware/wheelleg-infantry.cpp`
- `rmcs_ws/src/rmcs_bringup/config/wheelleg-infantry.yaml`
- `rmcs_ws/src/rmcs_core/src/hardware/device/dji_motor.hpp`
- `rmcs_ws/src/rmcs_core/src/broadcaster/value_broadcaster.cpp`

## Hard Blockers

### 1. Class name mismatch

In `oiopi_wheelleg_infantry.cpp`, the class defined in the file is:

- `rmcs_core::hardware::WheelLegInfantry`

But the exported plugin class is:

- `rmcs_core::hardware::OIOPIWheelLegInfantry`

The YAML and `plugins.xml` both try to load `rmcs_core::hardware::OIOPIWheelLegInfantry`.

This is inconsistent. The class identity must be unified across:

- the class definition,
- `PLUGINLIB_EXPORT_CLASS(...)`,
- `plugins.xml`,
- and the YAML file.

### 2. YAML parameter mismatch

The constructor in `oiopi_wheelleg_infantry.cpp` reads:

- `usb_pid_bottom_board`

But the current YAML only provides:

- `usb_pid_top_board: 0x93ac`

So the configured parameter does not match the code path that is actually used.

Also, `usb_pid_top_board` is misleading in the current implementation, because the `top_board_` creation code is commented out.

### 3. Incomplete hardware object construction

Inside `BottomBoard`, the file declares these members:

- `device::DmMotor chassis_hip_motors[4];`
- `device::LkMotor gimbal_yaw_motor_;`
- `device::DjiMotor bullet_feeder_motor_;`

but unlike the reference `wheelleg-infantry.cpp`, they are not initialized in the constructor initializer list.

This means the OIOPI hardware file is still incomplete as a concrete hardware class.

### 4. No control chain for the two wheel motors

The current OIOPI YAML loads only:

- `rmcs_core::hardware::OIOPIWheelLegInfantry -> infantry_hardware`

It does not load:

- a chassis controller,
- a PID chain,
- or any testing component that writes
  `/chassis/left_wheel/control_torque`
  and
  `/chassis/right_wheel/control_torque`

This matters because `device::DjiMotor` expects torque commands through:

- `name_prefix + "/control_torque"`

So even if the hardware plugin were fixed and loaded successfully, the motors still would not receive meaningful commands from the current YAML alone.

### 5. Status exists internally, but is not forwarded externally

The good part is that each `DjiMotor` already registers wheel outputs such as:

- angle
- velocity
- torque
- max_torque

So if the hardware component is alive and CAN feedback is arriving, motor status can exist inside the executor.

However, the current OIOPI YAML does not load `ValueBroadcaster`, so those values are not being forwarded to ROS topics for easy observation.

From the point of view of "I want to read motor status", the path is still incomplete.

## What Already Looks Reasonable

The intended two-motor path is clear in the source:

- two `M3508` motors are created for `/chassis/left_wheel` and `/chassis/right_wheel`
- left wheel command is sent on CAN1 frame `0x200`
- right wheel command is sent on CAN2 frame `0x200`
- left wheel feedback is read from CAN ID `0x201`
- right wheel feedback is read from CAN ID `0x202`

So the hardware idea itself is reasonable if your actual board wiring and motor IDs match this assumption.

## Final Judgment

If your current target is:

- command two `M3508` motors, and
- read their angle / velocity / torque status,

then the answer is:

Not yet.

The current three files are not enough in their present state.

## Minimum Changes Before Optimization

### A. Fix the plugin class identity

Make one class name consistent everywhere.

### B. Fix the board parameter wiring

Either:

- add `usb_pid_bottom_board` to the YAML,

or:

- change the hardware code so it uses the parameter you actually configured.

### C. Simplify the OIOPI hardware file

If your current goal is only two wheel motors, remove unrelated wheel-leg/gimbal/feeder parts for now, or fully initialize them.

### D. Add a real torque producer

You still need one of these:

- a minimal chassis controller that outputs `/chassis/left_wheel/control_torque` and `/chassis/right_wheel/control_torque`, or
- a temporary test component that writes those inputs directly.

### E. Add status forwarding

If you want to inspect feedback easily, load `ValueBroadcaster` and forward at least:

- `/chassis/left_wheel/angle`
- `/chassis/left_wheel/velocity`
- `/chassis/left_wheel/torque`
- `/chassis/right_wheel/angle`
- `/chassis/right_wheel/velocity`
- `/chassis/right_wheel/torque`

## Practical Recommendation

For your current stage, the shortest path is:

1. keep only the two wheel motors in the OIOPI hardware plugin
2. fix the class/export/plugin naming mismatch
3. provide the correct board parameter in YAML
4. add one minimal controller or test component that writes wheel torque
5. add `ValueBroadcaster` so you can verify wheel feedback

That is the fastest path to prove:

- command works
- CAN transmission works
- feedback works
- the two `M3508` motors can be driven and monitored

## Validation Note

I also tried:

- `colcon build --packages-select rmcs_core`

but a full package build could not be completed in this workspace state because installed dependency packages were missing:

- `rmcs_description`
- `rmcs_executor`
- `rmcs_msgs`
- `rmcs_utility`

So I could not use a successful build as the final proof. The conclusion above is based on direct source inspection.

## 2026-04-02 VS Code Include Resolution Changes

You asked what I changed for the VS Code red-underlined include problem.

I changed only editor configuration files.
I did not change the C++ source code for this part.

### 1. Added `.vscode/settings.json`

File:

- `/workspaces/RMCS/.vscode/settings.json`

Content added:

```json
{
    "clangd.arguments": [
        "--header-insertion=never",
        "--compile-commands-dir=/workspaces/RMCS/build"
    ],
    "C_Cpp.intelliSenseEngine": "disabled",
    "C_Cpp.default.compileCommands": "/workspaces/RMCS/build/compile_commands.json",
    "C_Cpp.default.compilerPath": "/usr/bin/c++",
    "C_Cpp.default.cppStandard": "c++20",
    "C_Cpp.default.intelliSenseMode": "linux-gcc-x64"
}
```

What this was meant to do:

- tell `clangd` to use `/workspaces/RMCS/build/compile_commands.json`
- tell the Microsoft C/C++ extension where the compile database is
- disable the Microsoft IntelliSense engine in this workspace, so only `clangd` is responsible for C++ diagnostics

### 2. Added root `.clangd`

File:

- `/workspaces/RMCS/.clangd`

Content added:

```yaml
CompileFlags:
  CompilationDatabase: /workspaces/RMCS/build
```

What this was meant to do:

- force `clangd` to find the compilation database even though `compile_commands.json` is not in a parent directory of the source files

## Why I Made These Changes

I checked the compile database and found that it already contains the correct include path for `librmcs`:

- `-I/workspaces/RMCS/rmcs_ws/src/rmcs_core/librmcs`

I also checked that the header file really exists:

- `/workspaces/RMCS/rmcs_ws/src/rmcs_core/librmcs/librmcs/client/cboard.hpp`

So for `#include <librmcs/client/cboard.hpp>`, the problem looked like a language-server configuration issue, not a missing file on disk.

## Why The Problem May Still Not Be Solved

If VS Code still shows the red underline, the likely reasons are:

### A. VS Code has not reloaded the workspace settings

The new files only affect the editor after:

- `Developer: Reload Window`
- `Clangd: Restart language server`

### B. You are not actually using `clangd`

If VS Code is still using the Microsoft C/C++ extension for diagnostics, my `.clangd` change will not help by itself.

### C. The current diagnostic is for a truly missing dependency, not just `librmcs`

For example, `fast_tf` include errors are different.
Those were real build errors before the submodule was pulled.

### D. The compile database may be stale

If the workspace changed after the last build, `compile_commands.json` may not match the current state until you build again.

## What I Did Not Change

I did not:

- modify `CMakeLists.txt`
- modify package dependencies
- change the `#include <librmcs/client/cboard.hpp>` line
- patch `rmcs_core` source files for this editor issue

## Current Direct Check Result

The direct facts I verified were:

- `/workspaces/RMCS/build/compile_commands.json` exists
- `/workspaces/RMCS/rmcs_ws/src/rmcs_core/librmcs/librmcs/client/cboard.hpp` exists
- the compile database contains the `librmcs` include directory

So if VS Code still underlines that specific include, the remaining problem is almost certainly on the editor / language-server side, not because the header path is absent from the build configuration.

## 2026-04-02 Clangd-Only Workspace Tightening

You then asked me to apply the stricter direct scheme instead of only describing it.

I changed two editor-side files:

- `/workspaces/RMCS/.vscode/settings.json`
- `/workspaces/RMCS/.vscode/extensions.json`

### 1. Updated `.vscode/settings.json`

I kept the previous `clangd` compile-database settings and added:

```json
"C_Cpp.errorSquiggles": "disabled"
```

The full intent of the current settings is:

- use `clangd` with `/workspaces/RMCS/build/compile_commands.json`
- disable Microsoft C/C++ IntelliSense
- disable Microsoft C/C++ error squiggles

This was meant to stop mixed diagnostics from two different C++ language engines.

### 2. Added `.vscode/extensions.json`

I added:

```json
{
    "recommendations": [
        "llvm-vs-code-extensions.vscode-clangd"
    ],
    "unwantedRecommendations": [
        "ms-vscode.cpptools"
    ]
}
```

This does not forcibly uninstall anything.
It only tells VS Code that this workspace should prefer:

- `clangd`

and should avoid:

- `ms-vscode.cpptools`

## What This Scheme Means

The workspace is now configured as:

- `clangd` as the intended C++ language server
- Microsoft C/C++ diagnostics turned off at the workspace level

So if you still see the same red underline after this change, the likely causes are:

- VS Code has not reloaded the new workspace settings
- `clangd` extension is not installed or not active
- the visible underline is from another extension
- the include error is real for a different dependency, not for `librmcs`

## Required User Actions After My Change

These steps are still required on the VS Code side:

1. `Developer: Reload Window`
2. `Extensions: Show Installed Extensions`
3. Confirm `clangd` is installed and enabled in this workspace
4. If `ms-vscode.cpptools` is installed, disable it for this workspace
5. Run `Clangd: Restart language server`

## Important Limitation

These changes only affect editor diagnostics.
They do not change:

- your CMake graph
- your ROS dependency graph
- your actual compiler

So if the build still fails, that is a separate problem from the red underline.

## 2026-04-02 OIOPI Source Parse Fix

After the include-resolution issue, a new VS Code diagnostic appeared around:

- `get_logger()`

The message was essentially saying that `OIOPIWheelLegInfantry` could not be treated correctly as an `rclcpp::Node` when calling the instance method `get_logger()`.

### What I changed

I changed this file:

- `/workspaces/RMCS/rmcs_ws/src/rmcs_core/src/hardware/oiopi_wheelleg_infantry.cpp`

Specifically, I fixed a missing semicolon in the constructor:

Before:

```cpp
bottom_board_ = std::make_unique<BottomBoard>(
    *this, *infantry_command_,
    static_cast<int>(get_parameter("usb_pid_bottom_board").as_int()))
```

After:

```cpp
bottom_board_ = std::make_unique<BottomBoard>(
    *this, *infantry_command_,
    static_cast<int>(get_parameter("usb_pid_bottom_board").as_int()));
```

### Why this matters

That missing semicolon breaks parsing of the constructor body.
Once parsing is broken, `clangd` often reports nonsense diagnostics at nearby lines.

So the `get_logger()` error was very likely a secondary parser artifact, not the true root problem.

### What I checked after the fix

I then tried to build `rmcs_core` again from the workspace root with:

```bash
colcon build --merge-install --packages-select rmcs_core
```

The next blocker was no longer the `get_logger()` parse issue.
The new build blocker was workspace dependency state:

- `/workspaces/RMCS/install/share/fast_tf/package.sh` missing
- `/workspaces/RMCS/install/share/serial/package.sh` missing

So after fixing the semicolon, the next real issue is that `fast_tf` and `serial` still need to be built into the current merged install tree before `rmcs_core` can build successfully.
