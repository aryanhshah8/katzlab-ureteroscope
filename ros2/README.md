# ROS 2 integration

Bringing the rig into ROS 2 so it can talk to Isaac Sim, RViz and MATLAB.

```
   [rig]  ──►  ┌────────┐  ◄──────►  [Isaac Sim]    digital twin, both directions
   Teensy      │  ROS2  │
   + katzlab   └────────┘
                 ▲    ▲
             [RViz]  [MATLAB]
```

**Target stack** — Linux Mint 22.x (Ubuntu 24.04 "noble") → **ROS 2 Jazzy**.
That is also the pairing NVIDIA recommends for the Isaac Sim ROS 2 bridge, so
this is the supported path rather than a workaround.

---

## 1. ROS 2 Jazzy

```bash
./ros2/setup-mint.sh
```

Then open a **new** terminal and check:

```bash
ros2 run demo_nodes_cpp talker      # one terminal
ros2 run demo_nodes_py listener     # another
rviz2
```

### Why a script rather than following the ROS guide directly

Every ROS 2 guide tells you to use `lsb_release -cs` for the apt source
codename. On Mint that returns `xia` -- a Mint release name no ROS repository
has heard of -- and apt fails with a 404 that reads like the server is down.
Mint publishes its Ubuntu base separately in `/etc/upstream-release/lsb-release`
and that is what has to be used. The script reads it, and refuses to run if the
base is not `noble`.

---

## 2. Isaac Sim

Requires an RTX GPU (present on this machine) and roughly 50 GB.

1. Install the proprietary NVIDIA driver via Mint's **Driver Manager**, reboot,
   confirm with `nvidia-smi`.
2. Install Isaac Sim per NVIDIA's current instructions -- the Omniverse Launcher
   was retired, so use the download or pip route from the docs.
3. **Source ROS before launching it**, in the same terminal:

   ```bash
   source /opt/ros/jazzy/setup.bash
   isaac-sim.sh
   ```

   Skipping this is the usual cause of the bridge failing with
   `librcutils.so: cannot open shared object file` -- Isaac falls back to its
   bundled libraries and they do not match the system ROS.

4. Enable the **ROS 2 Bridge** extension inside Isaac Sim.

Isaac Sim 6.x uses Python 3.12, which matches Jazzy -- the Python version
mismatch that plagued Isaac 5.1 with Jazzy is gone.

---

## 3. MATLAB

Needs the **ROS Toolbox**. Verify it can see the graph:

```matlab
ros2 node list
ros2 topic list
```

MATLAB ships its own DDS, so if it cannot see nodes that `ros2 topic list`
shows, the cause is almost always a mismatched `ROS_DOMAIN_ID` rather than
anything in MATLAB.

---

## 4. Everything must share a domain ID

```bash
export ROS_DOMAIN_ID=0
```

The setup script writes this into `~/.bashrc`. Isaac Sim, MATLAB and every
terminal must agree, or nodes simply do not see each other -- with no error.
It is the first thing to check when two things that should be talking are not.

---

## 5. The bridge node

Still to write: a node wrapping the existing `katzlab` package, which already
owns the serial protocol, calibration and safety interlocks. It is a wrapper,
not a rewrite.

Planned interface:

| direction | topic | type |
|---|---|---|
| publish | `/ureteroscope/joint_states` | `sensor_msgs/JointState` |
| publish | `/ureteroscope/laser_state` | custom or `std_msgs` |
| subscribe | `/ureteroscope/cmd_velocity` | per-axis velocity targets |
| service | `/ureteroscope/home`, `/ureteroscope/estop` | `std_srvs/Trigger` |

Joints: `linear` (prismatic, metres), `rotation` (revolute, radians),
`flexion` (revolute, radians). Note ROS uses **SI** -- the existing code is in
mm and degrees, so the node converts at the boundary rather than changing the
tuning that has already been dialled in on hardware.

The CSV recorder added earlier already captures exactly these fields, so the
data model is settled.

### Safety, unchanged

The laser interlocks stay in firmware and in `katzlab`. **Nothing about the
laser should ever be commandable from a ROS topic** -- a topic is
remote-triggerable by anything on the graph, including a simulator. Motion
over ROS, laser only from the physical controller.
