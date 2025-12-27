# rog_map

**Original repository:** <https://github.com/hku-mars/SUPER/tree/2ad3419c127a617c6d7df6925e81a14175a9c096/rog_map>

## Overview

`rog_map_ros.cpp` provides a ROS2 node wrapper (`rog_map::ROGMapROS`) around the core ROG-Map mapping functionality. It connects the ROG-Map system to ROS2 topics, timers, TF, and visualization so the map can be updated from incoming odometry and LiDAR (point-cloud) messages, and the local map state can be published for visualization and downstream modules.

This document focuses on practical aspects (functionality, inputs, outputs, key parameters and behaviors) and does not delve deeply into the underlying mapping algorithms.

## Key responsibilities

- Load configuration (via `cfg_path` parameter and `rog_map::Config`).
- Create TF helper (listener & broadcaster).
- Subscribe to odometry and point cloud topics (when `ros_callback_en` is enabled).
- Buffer point cloud frames and call the map update routine on a high-frequency timer.
- Publish visualization topics (occupied/unknown/frontier/esdf point clouds, marker arrays) according to configuration.
- Provide helper visualization utilities (bounding boxes, text, points).

## Inputs (subscriptions)

- **Odometry**
  - Topic: configured at `rog_map/ros_callback/odom_topic` (YAML name) — e.g. `lidar_odometry` in examples
  - Type: `nav_msgs/msg/Odometry`

- **Point Cloud**
  - Topic: configured at `rog_map/ros_callback/cloud_topic` — e.g. `registered_scan`
  - Type: `sensor_msgs/msg/PointCloud2`

## Outputs (publishers)

All visualization publishers use a QoS profile: best-effort, keep_last(1), volatile durability.

- `rog_map/occ` (PointCloud2) — Occupied cells inside the configured visualization range
- `rog_map/unk` (PointCloud2) — Unknown cells (optional, controlled by `pub_unknown_map_en`)
- `rog_map/inf_occ` (PointCloud2) — Inflated occupied cells
- `rog_map/inf_unk` (PointCloud2) — Inflated unknown cells
- `rog_map/frontier` (PointCloud2) — Frontier points (if frontier extraction enabled)
- `rog_map/esdf` (PointCloud2) — ESDF sampling/visualization (if `esdf_en` is true)
- `rog_map/map_bound` (MarkerArray) — Visual markers: visualization range box, local map box, raycast/update box, origin point, etc.

The node publishes the above topics only if the corresponding visualization flags are enabled and there is at least one active subscriber.

## Timers & Callbacks

- **Update timer** (when `ros_callback_en` is true): a wall timer is created and set to 1 ms interval (high-frequency). The timer calls `updateCallback()` which:
  - Pops the latest point cloud (protected by `rc_.updete_lock`), resets `unfinished_frame_cnt`, and calls `updateProbMap` (ROG-Map core function).
  - Logs timing information using `writeTimeConsumingToLog`.

- **Visualization timer** (if `cfg_.viz_time_rate > 0`): wall timer interval derived from `viz_time_rate` (Hz). Calls `vizCallback()` which publishes visualization outputs described above.

- Callbacks use mutually exclusive `rclcpp::CallbackGroup`s so odom/cloud/update do not run concurrently in the same group; a mutex additionally guards the point cloud buffer.

## Important parameters (YAML keys & behavior)

- `cfg_path` (node parameter) — Path to the YAML configuration file used by `rog_map::Config`.
- `rog_map/ros_callback/enable` (bool) — If **true**, the node subscribes to the configured odom and cloud topics and runs automatic updates; otherwise you must update the map manually with `updateMap`.
- `rog_map/ros_callback/cloud_topic` and `odom_topic` (string) — Topics for input data.
- `rog_map/ros_callback/odom_timeout` (double) — Max allowed time (s) since the last odom timestamp; if exceeded, incoming clouds are ignored.
- `rog_map/visualization/enable` (bool) — Turn on visualization publishers.
- `rog_map/visualization/frame_id` (string) — Frame id used for published messages and markers (e.g., `odom`). The TF transform broadcast uses this as parent frame.
- `rog_map/visualization/time_rate` (double) — Visualization timer frequency (Hz); if <= 0, visualization is not timer-driven.
- `rog_map/pub_unknown_map_en` (bool) — Publish unknown map clouds.
- `rog_map/esdf/enable` (bool) — Enable ESDF-related publishing.
- `rog_map/map_sliding/enable` and `map_sliding/threshold` — configure internal sliding map behavior (affects core mapping behavior).

For a full list of available map and raycasting parameters see `params/rog_map_params.yaml` shipped with the package (the node uses `rog_map::Config` to parse the YAML).

---

## Usage notes & tips

- Ensure odometry is published and up-to-date; otherwise point clouds are ignored (the node prints a warning and sets `map_empty_`).
- QoS for publishers/subscribers is set to best-effort and volatile durability for minimal latency (suitable for high-rate sensor streams).
- Visualization only happens when `visualization_en` is true and at least one subscriber is connected to the given topic; this avoids unnecessary work.
- The node is implemented as a composable component: it registers with `RCLCPP_COMPONENTS_REGISTER_NODE` and can be loaded into a component container.

---

## Implementation details (non-algorithmic)

- Threading model: the node uses several `rclcpp::CallbackGroup::MutuallyExclusive` callback groups to isolate odom, cloud and update operations. A mutex (`rc_.updete_lock`) protects the point-cloud buffer and `unfinished_frame_cnt`.
- The node converts `vec_E<Vec3f>` maps to `sensor_msgs/msg/PointCloud2` using `pcl::PointCloud<pcl::PointXYZ>` helper function `vecEVec3fToPC2`.
- Visualization helper functions: `visualizeBoundingBox`, `visualizeText`, and `visualizePoint` create `visualization_msgs::msg::Marker` and add them into `MarkerArray`.

---

## Example param snippet (from `params/rog_map_params.yaml`)

```yaml
rog_map:
  resolution: 0.1
  map_size: [50, 50, 6]
  ros_callback:
    enable: true
    cloud_topic: "registered_scan"
    odom_topic: "lidar_odometry"
    odom_timeout: 2.0
  visualization:
    enable: true
    range: [50, 50, 6]
    frame_id: "odom"
    time_rate: 10
```

---

## Acknowledgements & License

This code is derived from the original ROG-Map project by Yunfan REN et al. (MaRS Lab, University of Hong Kong). The original repository and commit referenced:

<https://github.com/hku-mars/SUPER/tree/2ad3419c127a617c6d7df6925e81a14175a9c096/rog_map>

The original license (GNU Lesser General Public License) and authorship are preserved; please keep the original license and author credits when reusing or republishing this code.
