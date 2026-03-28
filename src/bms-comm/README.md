# bms-comm

A ROS2 C++ package for publishing state information from a **JK-BMS** (Battery Management System). Designed for an **8S Li-Ion** setup (8,000mAh x 2 parallel configuration).

## Node: `bms_node`

### Parameters
- `serial_port` (default: `"/dev/ttyS0"`): The UART port connected to the BMS "GPS" port.
- `baud_rate` (default: `115200`): Communication speed.
- `publish_frequency` (default: `2.0`): Polling and publication rate in Hz.

### Topics
- `/bms/data` ([sensor_msgs/msg/BatteryState](https://docs.ros2.org/foxy/api/sensor_msgs/msg/BatteryState.html)):
    - Main battery state including total voltage, current, and SOC percentage.
    - `temperature` field maps to the internal MOSFET sensor.
    - `cell_voltage` array contains individual cell readings (8S).
- `/bms/temperature` ([std_msgs/msg/Float32MultiArray](https://docs.ros2.org/foxy/api/std_msgs/msg/Float32MultiArray.html)):
    - A 3-element array of all temperature sensors:
    - `Index 0`: Internal MOSFET temperature.
    - `Index 1`: Left external temperature probe.
    - `Index 2`: Right external temperature probe.

## Usage

### 1. Build the package
```bash
colcon build --packages-select bms-comm
```

### 2. Launch the node
```bash
ros2 launch bms-comm bms_launch.py serial_port:=/dev/ttyS0 publish_frequency:=5.0
```

## Information
- **frame_id**: All headers use `"bms_link"`.
- **Timestamps**: `header.stamp` reflects the time the measurement was received from the serial port.