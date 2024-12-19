# Stretch2 Cluster Navigation and Grasp Project

## Project Overview

This project enhances the functionality of the Stretch robot to:

- Detect object clusters using ROS services.
- Navigate to the detected cluster locations.
- Manipulate objects by moving the robotic arm and grasping objects using the gripper.

### Features

1. **Cluster Detection**:

   - Uses `/get_top1_cluster` to retrieve object clusters.
   - Processes clusters to select optimal target points (mean, max, min criteria).

2. **Navigation**:

   - Implements `/funmap/trigger_drive_to_scan` to reposition the robot when required.
   - Detects location changes to avoid redundant scans.

3. **Arm Manipulation**:

   - Moves the robotic arm to specific target points using `/funmap/move_arm`.
   - Grasps objects with the gripper using `/funmap/move_joints`.

4. **Error Handling**:

   - Handles obstructions by initiating head scans (`/funmap/trigger_head_scan`).
   - Retries failed actions with fallback strategies like random point selection.

---

## Directory Structure

```
stretch2_cluster_navigation_grasp/
├── detect_and_touch/
│   ├── scripts/
│   │   ├── detect_and_touch/
│   │   │   ├── poke.py                  # Moves the arm to a detected cluster point
│   │   │   ├── ros_object_query.py      # Queries clusters via ROS services
│   │   │   ├── location_change_detection.py # Detects if the robot's location has changed
│   │   │   ├── trigger_head_scan.py     # Performs head scans to handle obstructions
│   │   │   ├── stow_robot_example.py    # Demonstrates arm stowing via joint movements
├── stretch_srvs/
│   ├── srv/
│   │   ├── TriggerHS.srv                # Service definition for head scans
│   ├── CMakeLists.txt                   # Updated for new service definitions
├── README.md                             # Project description and instructions
```

---

## Installation

1. Clone this repository:

   ```bash
   git clone https://github.com/HAR5HA-7663/Stretch2_Cluster_navigation_grasp.git
   cd Stretch2_Cluster_navigation_grasp
   ```

2. Build the workspace:

   ```bash
   catkin_make
   source devel/setup.bash
   ```

3. Install dependencies:

   - Ensure the Stretch ROS stack is installed.
   - Install additional ROS packages as required for your project.

---

## Key Changes

### Updated Files:

1. `poke.py`: Improved arm manipulation logic with error handling.
2. `ros_object_query.py`: Added random fallback point strategy for robustness.
3. `CMakeLists.txt`: Integrated the `TriggerHS` service.

---

## Future Improvements

- Optimize navigation to avoid redundant head scans at unchanged locations.
- Add advanced point cloud filtering for better target selection.
- Integrate with additional ROS perception pipelines for enhanced cluster detection.
- Enhance gripper control logic for more reliable object grasping.
- Implement a logging system for better debugging and error tracking.

---

## Contributing

Feel free to contribute by opening issues or submitting pull requests:

1. Fork the repository.
2. Create a feature branch:
   ```bash
   git checkout -b feature/your-feature-name
   ```
3. Commit your changes and push:
   ```bash
   git push origin feature/your-feature-name
   ```
4. Open a pull request.

---


## Acknowledgments

Special thanks to the Stretch ROS community and Open Robotics for their contributions to robotics frameworks.

