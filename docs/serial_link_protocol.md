# Tao Robot Serial Link Protocol

This document defines the first lower-controller link tests between ROS on the Raspberry Pi / Sunrise X3 and the STM32 controller.

## Physical Link

- Controller side: STM32 USART2 on `PD5` / `PD6`.
- Computer side: Raspberry Pi 4B / Sunrise X3 40-pin header communication interface on pins `4`, `6`, `8`, `10`, described by the board documentation as `uart3`.
- Wiring rule: cross TX/RX, and always connect GND.
- Signal level: use 3.3 V TTL UART signals. Do not connect a 5 V serial signal directly to the Raspberry Pi UART pins.
- Current ROS default serial port: `/dev/ttyS0`, confirmed by Raspberry Pi 40-pin header loopback on physical pins `8` / `10` after disabling the serial console. If the board enables a separate UART overlay later, re-check the Linux device name before enabling real serial I/O.

## Stage 1: PING/PONG

Goal: prove that the Raspberry Pi and STM32 can exchange bytes reliably before migrating motion logic.

- ROS sends `PING`.
- STM32 replies `PONG`.
- Success means the serial device path, baudrate, wiring, and receive/transmit code are all basically correct.

ROS-side launch command after the UART device is confirmed:

```bash
roslaunch tao_bringup serial_test.launch port:=/dev/ttyS0 open_serial:=true
```

Note: the current STM32 firmware also streams binary telemetry frames on the same USART2 link. The ROS serial node must parse those frames as binary data and should not read the port with line-based ASCII decoding only.

## Current STM32 Telemetry Frame

The STM32 sends telemetry from `ROBOT_SendDataToRos()` through `UART2_SendPacket(comdata, 32, ID_STM2ROS_DATA)`.

- Baudrate: `115200`, `8N1`.
- Frame format: `AA 55` + 32-byte payload + 1-byte checksum + `7D` tail.
- Checksum: low 8 bits of the sum of all bytes before the checksum, including `AA 55` and the 32-byte payload.
- Payload endian: signed 16-bit big-endian values.

Payload layout:

| Index | Bytes | Meaning |
| --- | --- | --- |
| 0 | 0-1 | IMU accel X |
| 1 | 2-3 | IMU accel Y |
| 2 | 4-5 | IMU accel Z |
| 3 | 6-7 | IMU gyro X |
| 4 | 8-9 | IMU gyro Y |
| 5 | 10-11 | IMU gyro Z |
| 6 | 12-13 | velocity X, scaled by 1000 |
| 7 | 14-15 | velocity Y, scaled by 1000 |
| 8 | 16-17 | velocity W, scaled by 1000 |
| 9-14 | 18-29 | arm joint feedback values |
| 15 | 30-31 | battery voltage x100 |

The ROS topic `tao_serial/rx` currently publishes a readable `std_msgs/String` summary for validated telemetry frames.

## Stage 2: SET_SERVO

Goal: prove that one ROS command can trigger one safe hardware action.

Initial command shape:

```text
SET_SERVO <id> <angle>  
```

Example:

```text
SET_SERVO 1 90
```

## Migration Rule

Migrate one lower-controller feature at a time. Every migrated feature should have a matching ROS-side test command and a clear success signal.