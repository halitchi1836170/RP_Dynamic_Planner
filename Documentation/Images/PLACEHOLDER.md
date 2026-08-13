# Screenshot slots

Drop the images here using exactly these file names — the README links to them already.

| File | Suggested content |
| :--- | :--- |
| `01_overview.png` | Unity scene of the clinic + RViz2 with map and path side by side |
| `02_icp.png` | `/icp/path` (cyan) vs `/odometry/path` in RViz2 |
| `03_graph_slam.png` | `/graph_slam/nodes`, `/graph_slam/loop_closure_edges`, `/graph_slam/cone_fan` |
| `04_occupancy_grid.png` | Final `/occupancy_grid` and/or the generated `occupancyGridMap.pgm` |
| `05_distance_map.png` | `/distanceMap` showing the gradient away from walls |
| `06_path_smoothing.png` | `/planned_path` (raw A*) overlaid with `/smoothed_path` (LOS-PS) |
| `07_control.png` | Robot tracking `/splined_path` |
| `08_localization.png` | `/debug/localization/icp_pose` vs `/debug/current_pose` vs `/debug/true_pose` |
| `09_dynamic_replanning.png` | Moving obstacle on the path and the replanned trajectory (once implemented) |
