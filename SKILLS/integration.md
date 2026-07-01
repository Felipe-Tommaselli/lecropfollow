# ROS ↔ TD-MPC2 Integration Guide

This document maps every interface point between the ROS/Gazebo simulation stack
and the TD-MPC2 reinforcement learning agent. Use it as the contract when plugging
in a different robot or perception system.

---

## Architecture Overview

```
┌─────────────────────────────────────────────────────────┐
│                    Gazebo Simulation                     │
│                                                         │
│  ┌──────────────┐   /robot/ground_truth [Odometry]      │
│  │  Robot URDF  │──────────────────────────────────►    │
│  │ (diff-drive) │   /robot/odom        [Odometry]       │
│  │              │◄──────────────────── /robot/cmd_vel   │
│  └──────────────┘          [Twist]                      │
│                                                         │
│  ┌──────────────┐   /robot/distance_error [Float32MA]   │
│  │ rowfollow_gt │──────────────────────────────────►    │
│  │  (GT node)   │   /robot/heading_error  [Float32MA]   │
│  └──────────────┘                                       │
│                                                         │
│  ┌──────────────┐   /robot/vision/keypoint_heatmap      │
│  │  Perception  │──────────────────────────────────►    │
│  │  node / stub │   /robot/vision/keypoint_vis_argmax/  │
│  └──────────────┘       compressed  [CompressedImage]   │
└───────────────────────────────┬─────────────────────────┘
                                │  ROS topics
                                ▼
┌─────────────────────────────────────────────────────────┐
│              tdmpc2/envs/gazebo.py  (GazeboEnv)         │
│                                                         │
│  Observation assembly → agent.act() → cmd_vel publish   │
└─────────────────────────────────────────────────────────┘
```

---

## Required ROS Topics

All topics are namespaced under `robot_ns` (default: `robot`).
Set `robot_ns` in `tdmpc2/config.yaml` to match your robot.

### Subscribed by `GazeboEnv` (tdmpc2 reads these)

| Topic | Message Type | Rate | What it provides |
|---|---|---|---|
| `/<ns>/ground_truth` | `nav_msgs/Odometry` | ≥50 Hz | Robot pose and velocity. `pose.pose.position.x` → travel distance; `twist.twist.linear.x` → actual forward speed (for stuck detection); `twist.twist.angular.*` → orientation rates |
| `/<ns>/distance_error` | `std_msgs/Float32MultiArray` | ≥10 Hz | `data[0]` = signed lateral offset from crop row centre (m). Positive = robot is right of centre |
| `/<ns>/vision/keypoint_heatmap` | `std_msgs/Float32MultiArray` | ≥10 Hz | **state obs mode**: CNN heatmap features. Shape: 13440 floats (3 channels × 56 H × 80 W). Forms indices [2:] of the observation vector |
| `/<ns>/vision/keypoint_vis_argmax/compressed` | `sensor_msgs/CompressedImage` | ≥10 Hz | **rgb obs mode**: annotated keypoint visualisation image. Resized to `image_size × image_size` px |

### Published by `GazeboEnv` (tdmpc2 writes these)

| Topic | Message Type | When | What it sends |
|---|---|---|---|
| `/<ns>/cmd_vel` | `geometry_msgs/Twist` | Every `step()` | Robot velocity command. `linear.x` ∈ [0.1, 1.0] m/s; `angular.z` ∈ [−0.9, 0.9] rad/s after denormalisation. An EMA filter (α=0.3) is applied before publishing |
| `/<ns>/collision` | `std_msgs/String` | Episode end | Collision type string: `'stuck'`, `'distance left'`, `'distance right'`, `'acrobatic'` |
| `/<ns>/max_x_position` | `std_msgs/Float32` | Episode end | Maximum forward travel reached in the episode (m) |

---

## Gazebo Services Used

These are standard Gazebo services — no changes needed for a new robot.

| Service | Type | When called |
|---|---|---|
| `/gazebo/unpause_physics` | `std_srvs/Empty` | Before each `step()` action executes; during `reset()` settling |
| `/gazebo/pause_physics` | `std_srvs/Empty` | After each `step()` action executes; after `reset()` settling |
| `/gazebo/reset_world` | `std_srvs/Empty` | At the start of each `reset()` call |

---

## Observation Space

Controlled by `obs` in `tdmpc2/config.yaml`.

### `obs: state` (default)

```
index 0:    filtered_action[0]   — commanded linear velocity  (post-EMA, m/s)
index 1:    filtered_action[1]   — commanded angular velocity (post-EMA, rad/s)
index 2–13441: keypoint_heatmap  — 13440 floats from /<ns>/vision/keypoint_heatmap
```

Total: 13442 floats → `obs_shape: 13442` in config.yaml.

> **Note:** `reset()` uses `[vel_x, yaw]` for the first two values instead of the
> filtered action. This inconsistency is preserved from the original implementation.

### `obs: rgb`

A `(3, image_size, image_size)` float32 array sourced from
`/<ns>/vision/keypoint_vis_argmax/compressed` and resized to `image_size` px
(default 128). Set `obs_shape` accordingly.

---

## Action Space

- **Raw space:** `gym.spaces.Box([-1, -1], [1, 1])` — both axes normalised to [−1, 1]
- **Denormalised before publishing:**
  - `linear_x  = (action[0] + 1) / 2 * (1.0 − 0.1) + 0.1`  → [0.1, 1.0] m/s
  - `angular_z = (action[1] + 1) / 2 * (0.9 − (−0.9)) + (−0.9)` → [−0.9, 0.9] rad/s
- **EMA smoothing:** `filtered = 0.3 × action + 0.7 × filtered_prev`
- Published as `geometry_msgs/Twist` on `/<ns>/cmd_vel`

---

## Plugging in a Different Robot

### Step 1 — Set the namespace

In `tdmpc2/config.yaml`:
```yaml
robot_ns: my_robot    # must match the topic prefix your robot uses
```

### Step 2 — Implement the required topics

Your robot (or a ROS adapter node) must publish/subscribe the topics listed above
under the same namespace. The `ground_truth` odometry is the minimum required to
run the environment loop. The `distance_error` topic defines the task signal.

### Step 3 — Swap the URDF (simulation only)

Replace `src/robot_description/urdf/robot.urdf.xacro` with your robot model.
The minimal model uses `libgazebo_ros_diff_drive.so` and `libgazebo_ros_p3d.so`
(both ship with `ros-noetic-gazebo-dev`). The launch file passes `robot_ns` as
a xacro argument.

### Step 4 — Replace the perception stub (optional)

`src/terra_utils/scripts/perception_stub.py` publishes zero-filled heatmap
features. Replace it with a node that runs your keypoint or feature extraction
model and publishes on `/<ns>/vision/keypoint_heatmap`.

Update `terra_gazebo/launch/terra_gazebo.launch` to launch your perception node
instead of `perception_stub`.

---

## Scenario / World

World files and 3D models are in `src/terra_worlds/`:

```
src/terra_worlds/
  worlds/
    farm.world          ← default scenario (crop rows, obstacles)
  models/
    corn_plot_*/        ← corn field arrangements
    sorghum_plot_*/     ← sorghum variants
    tobacco_plot_*/     ← tobacco variants
    tdmpc_*/            ← crop-row corridor models used in training
    heightmap_*/        ← terrain heightmaps
```

To use a different world, pass `world:=<name>.world` to `terra_gazebo.launch` or
change the `launchfile` entry and world path in your launch file.

The `terra_worlds` package export sets `GAZEBO_MODEL_PATH` automatically when
sourcing the catkin workspace (`devel/setup.bash`). This is also set explicitly
in `terra_gazebo.launch` for convenience when not using `catkin_make`.

---

## Packages at a Glance

| Package | Path | Role |
|---|---|---|
| `robot_description` | `src/robot_description/` | Minimal diff-drive URDF — **replace for your robot** |
| `terra_gazebo` | `src/terra_gazebo/` | Launch files — world + robot spawn + utility nodes |
| `terra_worlds` | `src/terra_worlds/` | Gazebo world file and 3D crop models |
| `terra_utils` | `src/terra_utils/` | `rowfollow_gt.py` (GT errors) + `perception_stub.py` |
