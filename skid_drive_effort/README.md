# skid_drive_effort

ROS 2 node that publishes a **left/right drive effort proxy** for skid-steer and
differential-drive robots.

The node estimates an *effective drive effort* signal by inverting a simple,
control-oriented body dynamics model using:

- wheel angular velocities (`sensor_msgs/JointState.velocity`)
- yaw rate from IMU (`sensor_msgs/Imu.angular_velocity.z`)

The resulting signal is useful for:
- actuation saturation detection
- command feasibility checks
- controller supervision / fallback logic
- resilient control on heavy robots

---

## Output interface

Publishes:

/drive_effort_lr std_msgs/Float32MultiArray
data[0] = left drive effort
data[1] = right drive effort


**Units:** model-consistent *effective drive effort*.  
This is **not guaranteed to be physical motor torque (Nm)** unless the backend
provides real torque telemetry.

---

## Tested platforms

- Clearpath Jackal (ROS 2)
  - `/platform/joint_states.effort` is NaN
  - wheel speeds + IMU yaw rate are available
  - node provides a consistent effort proxy for supervision logic

---

## Usage

### Build
```bash
cd ~/ros2_ws
colcon build --packages-select skid_drive_effort
source install/setup.bash
```

### Run
```
ros2 launch skid_drive_effort drive_effort_proxy.launch
```