# WheelLeg 2.0 Migration Notes

## What I changed
- `wheelleg-infantry-ai.cpp` is rewritten against the new board API style used by `steering-infantry.cpp`.
- `TopBoard` and `BottomBoard` now use `librmcs::agent::CBoard`.
- CAN/UART/DBUS/IMU callbacks are changed to the `librmcs::data::*View` form.
- CAN sending is changed from `TransmitBuffer` to `start_transmit()`.

## Assumptions I kept on purpose
- `TopBoard` code is migrated, but I kept the current runtime behavior: it is still not instantiated in the constructor.
  This avoids changing the current wheelleg runtime path while still updating the code for the new system.
- `DmMotor` is kept unchanged as requested.
  In `wheelleg-infantry-ai.cpp`, I only adapt DM motor TX/RX locally by converting between `uint64_t` and byte spans.
- Topic names such as `/gimbal/yaw/velocity` and `/chassis/yaw/velocity` are kept in the old form.
  I did not rename them to `*_imu`, because that would require coordinated changes in controller and YAML files outside the allowed edit scope.
- The local fork's `rmcs_utility::RingBuffer` still exposes `pop_front_multi` and `emplace_back_multi`.
  The steering reference uses `*_n`, but I did not change the utility class because you restricted edits to this file and this note.

## Open opinions
- `TopBoard` can be re-enabled later by creating it in the constructor once you confirm the actual top-board hardware path is ready again.
- `BottomBoard` still keeps the original wheelleg command behavior for yaw motor and bullet feeder.
  I did not change that logic here, because that is a functional design choice, not only a board-API migration.

## About the local byte adapters
- `as_byte_span()` exists because this wheelleg file is in a transition state.
  The new `agent::CBoard` transmit API wants `std::span<const std::byte>`, but the motor helpers currently used in this fork still return old raw values such as `uint16_t` and `uint64_t`.
  So `as_byte_span()` is a small local adapter that wraps those raw command values as bytes for `builder.can1_transmit()` and `builder.can2_transmit()`.
- `can_data_to_u64()` exists for the opposite direction.
  In the new callback API, received CAN data arrives as `data.can_data`, which is already a byte span.
  But the old wheelleg-side motor wrappers in this fork still use `store_status(uint64_t)` for `DjiMotor`, `LkMotor`, and `DmMotor`.
  So `can_data_to_u64()` converts the received byte span back into the old `uint64_t` form expected by those wrappers.
- These two helpers do not appear in the steering example because the steering example is already on the newer motor helper layer.
  There, motor commands already expose `.as_bytes()` and motor status methods already accept byte spans directly, so no conversion shim is needed.
- When wheelleg's `DjiMotor` / `LkMotor` / `DmMotor` helpers are upgraded to the same interface as the newest system, both helpers can be deleted.
