# QPin + dq_vizlab Simulation Environment

This directory is a standalone ROS 2 simulation scaffold for testing the
`dq_vizlab` absolute-pose planner against a TF/Gazebo robot visualization.
The ROS-side structure follows the lightweight approach used by
`/home/syx/qpin_grasp_demo`, while the robot model is the `dq_vizlab`
`sailor_r1_pro_description` URDF so the solver and simulation use the same
`base_link -> body_link -> lee_link/ree_link` chains.

## Layout

- `assets/sailor_r1_pro_description/`: URDF and STL meshes copied from `dq_vizlab`.
- `launch/robot_tf.launch.py`: minimal robot TF + RViz simulation.
- `launch/gazebo_visual.launch.py`: Gazebo scene plus TF-driven robot visuals.
- `launch/dq_absolute_demo.launch.py`: starts the simulation and runs one absolute-pose plan.
- `scripts/joint_state_source.py`: publishes `/qpin_sim/joint_states` and accepts trajectory commands.
- `scripts/gazebo_robot_visual_follower.py`: mirrors TF links into Gazebo visual models.
- `scripts/dq_absolute_plan_demo.py`: calls `dq_vizlab` `plan_absolute_pose(...)` and publishes a `JointTrajectory`.
- `worlds/qpin_visual_scene.world`: simple Gazebo world with ground, base frame marker, and box.

## TF / Topic Conventions

The root frame is `base_link`, matching `dq_vizlab`.

- Dynamic TF: `/qpin_sim/tf`
- Static TF: `/qpin_sim/tf_static`
- Joint states: `/qpin_sim/joint_states`
- Robot description: `/qpin_sim/robot_description`
- Direct joint command input: `/qpin_sim/joint_command`
- Trajectory input: `/qpin_sim/joint_trajectory`
- World pose input for Gazebo visuals: `/qpin_sim/world_from_base`

Important solver links:

- `base_link`
- `body_link`
- `lee_link`
- `ree_link`

## Launch

Minimal RViz/TF:

```bash
source /opt/ros/humble/setup.bash
ros2 launch /home/syx/qpin_sim_env/launch/robot_tf.launch.py
```

Gazebo visual scene:

```bash
source /opt/ros/humble/setup.bash
ros2 launch /home/syx/qpin_sim_env/launch/gazebo_visual.launch.py gui:=true rviz:=true
```

Run the dq_vizlab absolute planner demo in the simulation:

```bash
source /opt/ros/humble/setup.bash
ros2 launch /home/syx/qpin_sim_env/launch/dq_absolute_demo.launch.py gui:=true rviz:=true dz:=-0.03 waypoint_count:=4
```

Headless:

```bash
source /opt/ros/humble/setup.bash
ros2 launch /home/syx/qpin_sim_env/launch/dq_absolute_demo.launch.py gui:=false rviz:=false visual_follower:=false
```

If another Gazebo/ROS simulation is already running, isolate this one:

```bash
source /opt/ros/humble/setup.bash
ROS_DOMAIN_ID=42 GAZEBO_MASTER_URI=http://127.0.0.1:11347 \
  ros2 launch /home/syx/qpin_sim_env/launch/dq_absolute_demo.launch.py gui:=false rviz:=false
```

Run the OMPL joint-path plus dq_vizlab retime demo:

```bash
source /opt/ros/humble/setup.bash
source /home/syx/sailor_rviz_ws/install/setup.bash
ros2 launch /home/syx/qpin_sim_env/launch/dq_ompl_demo.launch.py gui:=false rviz:=false visual_follower:=false
```

## Absolute Pose Demo

By default `dq_absolute_plan_demo.py` reads the current `/qpin_sim/joint_states`,
computes the current absolute pose with `NullspaceSolverClient`, shifts the
absolute target by `dx/dy/dz`, plans with `AbsolutePoseTOPPRAPlanner`, and
publishes the retimed joint trajectory to `/qpin_sim/joint_trajectory`.

Run only the planner/publisher against an already-running simulation:

```bash
source /opt/ros/humble/setup.bash
python3 /home/syx/qpin_sim_env/scripts/dq_absolute_plan_demo.py --dz -0.03 --waypoint-count 4 --sample-count 80
```

Or give a concrete absolute translation while preserving the current absolute
orientation:

```bash
python3 /home/syx/qpin_sim_env/scripts/dq_absolute_plan_demo.py \
  --target-x 0.45 --target-y 0.0 --target-z 1.0 --waypoint-count 4
```

For a complete absolute dual-quaternion target:

```bash
python3 /home/syx/qpin_sim_env/scripts/dq_absolute_plan_demo.py \
  --target-dq "1,0,0,0,0,0.225,0,0.5"
```

The published trajectory contains the 20 `dq_vizlab` humanoid joints. The
simulated joint-state source also publishes the two wheel joints, but it accepts
partial `JointTrajectory` commands containing only the humanoid joints.

## OMPL + dq_vizlab Demo

`dq_ompl_joint_plan_demo.py` uses `dq_vizlab` to solve one absolute target pose
into a final joint goal, calls `sailor_r1_pro_ompl/joint_space_demo` to generate
an OMPL joint path from the current `q` to that goal, captures the published
joint path, retimes it with `dq_vizlab.retime_waypoints(...)`, and then
publishes the resulting `JointTrajectory` to `/qpin_sim/joint_trajectory`.

Run only the planner/publisher against an already-running simulation:

```bash
source /opt/ros/humble/setup.bash
source /home/syx/sailor_rviz_ws/install/setup.bash
python3 /home/syx/qpin_sim_env/scripts/dq_ompl_joint_plan_demo.py --dz -0.03 --solve-time 0.3 --sample-count 20
```

## Manual Joint Commands

Publish a one-shot joint command:

```bash
ros2 topic pub --once /qpin_sim/joint_command sensor_msgs/msg/JointState \
"{name: ['ljoint1', 'rjoint1'], position: [-0.4, 0.4]}"
```

Publish a simple trajectory:

```bash
ros2 topic pub --once /qpin_sim/joint_trajectory trajectory_msgs/msg/JointTrajectory \
"{joint_names: ['ljoint1', 'rjoint1'], points: [
  {positions: [-0.52, 0.52], time_from_start: {sec: 0, nanosec: 0}},
  {positions: [-0.30, 0.30], time_from_start: {sec: 2, nanosec: 0}}
]}"
```

## Notes

`gazebo_visual.launch.py` uses `robot_state_publisher` as the source of truth
for TF. The Gazebo visual follower spawns each URDF visual link as a kinematic
Gazebo model and updates it from `/qpin_sim/tf`, following the same idea as
`qpin_grasp_demo/sim/qpin_gazebo_linked_sim.launch.py`.
