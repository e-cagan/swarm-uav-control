+++markdown
# SWARM MISSION BOOTSTRAP  
*(ROS 2 Humble + MAVROS + PX4 SITL)*  

**Tested OS:** Ubuntu 22.04 (Jammy)  

---

## 📜 Overview
- Launch PX4 SITL (Gazebo Classic) with **3 plane model vehicles**.
- Start one **MAVROS** node per vehicle, each under its own ROS 2 namespace (`/uas1`, `/uas2`, `/uas3`).
- *(Optional)* Use QGroundControl for mission upload/viewing.
- Run a **single ROS 2 node** (`swarm_pkg/swarm_control_node`) to synchronously:
  - Set all vehicles to `AUTO.MISSION`
  - Arm them together
- Designed for **quick demos** and **repeatable lab tests**.

---

## 📂 Repository Layout (minimal)
```
swarm_ws/
  src/
    swarm_pkg/
      package.xml        (ament_python)
      setup.py
      swarm_pkg/
        swarm_control_node.py   (main script)
```

---

## ⚙️ System Requirements
- Ubuntu 22.04
- ROS 2 Humble installed & sourced (`/opt/ros/humble/setup.bash`)
- MAVROS and `mavros_msgs` for ROS 2
- PX4-Autopilot with Gazebo Classic SITL tools
- QGroundControl *(optional)*

---

## 🛠 Build Instructions
1. **Source ROS 2 environment:**
   ```bash
   source /opt/ros/humble/setup.bash
   ```
2. **Build workspace:**
   ```bash
   cd ~/swarm_ws
   colcon build --symlink-install
   source install/setup.bash
   ```

---

## 🚀 Run Book (Terminals)
**Terminal 1 – PX4 SITL:**
```bash
./Tools/simulation/gazebo-classic/sitl_multiple_run.sh -n 3 -m plane
```
**PX4 UDP Ports:**
```
Instance 1: 14581
Instance 2: 14582
Instance 3: 14583
```

**Terminal 2 – MAVROS /uas1:**
```bash
ros2 run mavros mavros_node --ros-args -r __ns:=/uas1 \
  -p fcu_url:="udp://:14540@127.0.0.1:14581" \
  -p gcs_url:="tcp-l://0.0.0.0:5760" \
  -p tgt_system:=2 -p tgt_component:=1
```

**Terminal 3 – MAVROS /uas2:**
```bash
ros2 run mavros mavros_node --ros-args -r __ns:=/uas2 \
  -p fcu_url:="udp://:14541@127.0.0.1:14582" \
  -p gcs_url:="tcp-l://0.0.0.0:5761" \
  -p tgt_system:=3 -p tgt_component:=1
```

**Terminal 4 – MAVROS /uas3:**
```bash
ros2 run mavros mavros_node --ros-args -r __ns:=/uas3 \
  -p fcu_url:="udp://:14542@127.0.0.1:14583" \
  -p gcs_url:="tcp-l://0.0.0.0:5762" \
  -p tgt_system:=4 -p tgt_component:=1
```

**Terminal 5 – QGroundControl (optional):**
```bash
./QGroundControl-x86_64.AppImage
```

**Terminal 6 – Swarm Mission Controller:**
```bash
ros2 run swarm_pkg swarm_control_node --ros-args \
  -p vehicle_namespaces:="['uas1','uas2','uas3']" \
  -p mission_mode:=AUTO.MISSION \
  -p service_timeout_s:=30.0 \
  -p wait_connected_s:=30.0 \
  -p sync_delay_s:=2.0 \
  -p confirm_sleep_s:=3.0
```

---

## 🤖 Controller Logic
- Checks each namespace has reachable MAVROS services & `/state` reports `connected=true`.
- Waits for all ready (or timeout).
- Synchronously:
  1. Sets mode to `AUTO.MISSION`
  2. Arms all vehicles
- Prints final status (connected, armed, mode) for each.

---

## 📌 Parameters
| Parameter             | Type      | Default         | Description |
|-----------------------|-----------|-----------------|-------------|
| `vehicle_namespaces`  | list[str] | `['uas1','uas2','uas3']` | MAVROS namespaces |
| `mission_mode`        | string    | `AUTO.MISSION`  | PX4 mode |
| `service_timeout_s`   | float     | 30.0            | Service readiness timeout |
| `wait_connected_s`    | float     | 30.0            | Wait for `/state connected=true` |
| `sync_delay_s`        | float     | 2.0             | Delay before issuing commands |
| `confirm_sleep_s`     | float     | 3.0             | Delay before confirming |

---

## 🔌 Port & ID Mapping (example)
| Namespace | Local Bind Port | PX4 Port | tgt_system | GCS Port |
|-----------|-----------------|----------|------------|----------|
| `/uas1`   | 14540           | 14581    | 2          | 5760     |
| `/uas2`   | 14541           | 14582    | 3          | 5761     |
| `/uas3`   | 14542           | 14583    | 4          | 5762     |

---

## ✅ Quick Checks
**Check MAVROS connection:**
```bash
ros2 topic echo /uas1/state
```
Expect:  
`connected: true`, `mode: AUTO.LOITER` (before mission mode switch).

**List services:**
```bash
ros2 service list | grep uas1 | grep mavros
```

**Service types:**
```bash
ros2 interface show mavros_msgs/srv/CommandBool
ros2 interface show mavros_msgs/srv/SetMode
```

---

## ⚠️ Safety Notes
- Gazebo Classic runs headless in PX4 script → use QGC or ROS topics to monitor.
- Keep QGC open for health & pre-arm check feedback.
- This demo **arms all vehicles simultaneously** → use only in simulation or safe environments.

---

## 📄 License
MIT

---

## 🙌 Credits
- PX4-Autopilot project & MAVROS maintainers
- ROS 2 community

---

## 📝 Changelog
- **Initial version**: synchronous mission mode + arming for N vehicles with confirmation logic.
+++
