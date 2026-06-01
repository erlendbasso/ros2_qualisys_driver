# ros2_qualisys_driver

ROS 2 driver for streaming Qualisys QTM 6DOF rigid body poses as
`geometry_msgs/msg/PoseStamped` messages.

The node connects to QTM, reads the configured 6DOF bodies, starts a real-time
stream, and publishes one pose topic per tracked body.

## Requirements

- ROS 2 Humble
- QTM reachable from the ROS host
- QTM real-time server enabled
- 6DOF bodies configured in QTM

## Build

From a ROS 2 workspace:

```bash
cd ~/ros2_ws/src
git clone git@github.com:erlendbasso/ros2_qualisys_driver.git
cd ..
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select ros2_qualisys_driver
source install/setup.bash
```

## Configure

Edit `params/params.yaml` before launching:

```yaml
/**:
  ros__parameters:
    server_address: "192.168.0.41"
    base_port: 22222
    update_period_ms: 10
    receive_timeout_us: 1000
    udp_port: 14570
    frame_id: "qualisys"
    topic_prefix: "qualisys"
    qtm_minor_protocol_version: 21
```

| Parameter | Default | Description |
| --- | --- | --- |
| `server_address` | `127.0.0.1` | QTM server IP address or host name. |
| `base_port` | `22222` | QTM real-time base TCP port. Must be `1-65535`. |
| `update_period_ms` | `10` | ROS timer period for polling QTM packets. Must be positive. |
| `receive_timeout_us` | `1000` | Per-receive timeout in microseconds. Use a small value to avoid blocking the lifecycle timer. |
| `udp_port` | `14570` | UDP port for QTM stream data. Use `0` or `-1` for TCP. UDP ports must be `1024-65535`. |
| `frame_id` | `qualisys` | Frame ID assigned to published pose headers. |
| `topic_prefix` | `qualisys` | Relative topic prefix for generated pose topics. Leading and trailing slashes are stripped. |
| `qtm_minor_protocol_version` | QTM SDK default | Minor version of the QTM RT protocol to request. The bundled default is currently `21`; try `17` for older QTM servers. |

## Run

```bash
ros2 launch ros2_qualisys_driver qualisys.launch.py
```

The launch file starts the lifecycle node, sends the configure transition, and
activates it once configuration succeeds.

## Published Topics

Each QTM 6DOF body gets a pose topic:

```text
<topic_prefix>/<sanitized_subject_name>/pose
```

For example, with `topic_prefix: "qualisys"` and a QTM body named `rb5`, the
driver publishes:

```text
qualisys/rb5/pose
```

QTM subject names are sanitized into ROS topic tokens. Non-alphanumeric
characters are converted to `_`, and names that do not start with a letter or
`_` are prefixed with `_`. If two QTM names sanitize to the same token, the
driver appends a numeric suffix to keep the topics distinct.

The default launch file remaps:

```text
qualisys/rb5/pose -> /mavros/vision_pose/pose
```

Adjust or remove this remap if your rigid body is named differently.

## Lifecycle Behavior

The driver is a ROS lifecycle node.

- `configure` validates parameters, connects to QTM, reads 6DOF and system
  settings, and starts QTM frame streaming.
- `activate` starts the receive timer.
- `deactivate` stops the receive timer and deactivates pose publishers.
- `cleanup` clears publishers and disconnects from QTM.
- `shutdown` stops streaming and disconnects if connected.

Pose publishers are activated only while their subject is tracked. If QTM sends
non-finite pose data for a subject, the driver marks that subject as lost and
deactivates its publisher until valid data returns.

## UDP vs TCP

Set `udp_port` to a valid unprivileged UDP port (`1024-65535`) to have QTM stream
data over UDP. Set `udp_port` to `0` or `-1` to use TCP.

UDP is usually preferable for real-time pose streams, but it requires that QTM
can send packets to the selected port on the ROS host. Check host firewall rules
and container networking if no data arrives.

## Docker

This repository includes a simple ROS Humble Docker setup:

```bash
docker compose build ros2_qualisys
docker compose run --rm ros2_qualisys /bin/bash
```

To validate the package inside the container:

```bash
docker compose run --rm ros2_qualisys /bin/bash -lc \
  'source /opt/ros/humble/setup.bash && cd /workspaces/ros2_ws && colcon build --packages-select ros2_qualisys_driver'
```

On Apple Silicon hosts, Docker may warn that the image is `linux/amd64` while the
host is `linux/arm64/v8`. The build still works under emulation, but it is slower.

## Troubleshooting

- `Connection to QTM server failed`: check `server_address`, `base_port`, network
  routing, and whether the QTM real-time server is enabled.
- `Reading 6DOF body settings failed`: confirm that QTM has 6DOF bodies
  configured and that the requested protocol version is supported.
- No pose topics appear: verify that the lifecycle node reached the active state
  and that QTM is actively streaming tracked 6DOF data.
- No messages on an expected topic: check the sanitized topic name and any launch
  remaps. A body named `rb-5` publishes under `rb_5`.
- UDP mode has no data: verify the selected `udp_port`, firewall rules, and
  container network mode. Try TCP mode with `udp_port: 0` to isolate networking
  issues.

## Notes

- QTM reports position in millimeters; the driver publishes position in meters.
- The driver maps QTM rotation matrices explicitly as Eigen column-major matrices
  before converting them to quaternions.
- `package.xml` still contains a placeholder license declaration. Choose and add
  the project license before publishing releases.
