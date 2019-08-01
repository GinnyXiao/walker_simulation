# Walker_simulation

Pick-and-place simulation for wheeled walker robot.

## Getting Started

1. Launch move_group: ```roslaunch walker_simulation move_group.launch```
2. Run Rviz and use this config: ```walker_simulation.rviz```
3. Run conveyor simulator: ```rosrun walker_simulation conveyor_simulator```
4. Run demo: ```rosrun walker_simulation walker_simulation_node```

## Demo Pipeline

#### Hardcoded variables

- an upper bound on the time to plan and execute the motion to a grasp location: `time_to_reach_grasp = 4`
- `allowed_planning_time = 3`
- workspace boundaries (w.r.t. the tool frame) to ensure that the goal is not too far away from the arm: `min_workspace_y` and `max_workspace_y`

#### Work pipeline

1. Estimate object velocity: Pick an object that is present, calculate its average velocity from the recorded list of its history positions (from ar marker msgs).
2. Compute predicted goal: Calculate how far the object will have moved during the time it takes to plan and execute the motion to grasp location by multiplying hardcoded time to reach grasp with the estimated object velocity `offset = time_to_reach_grasp * object_vel`. The predicted goal would be `object_current_pose + offset`. If the predicted goal is ahead of `min_workspace_y`, then set it to `min_workspace_y`. If it falls within the workspace, then it's what it is. Otherwise we skip grasping this object because the object is moving too fast or it's too close there's no enough time for grasping. For the last case, come back to stage 1.
3. Once we have the predicted goal, the actual time available for planning and grasping can be calculated as `time_to_grasp = (goal - current_pose) / object_velocity`.
4. Plan for the predicted pose and then execute the plan. Record planning and execution duration `pre_grasp_duration = time_execution_done - time_goal_ready`.
5. Calculate how long the robot should wait before closing the hand. The waiting time would be `time_to_grasp - pre_grasp_duration`.
6. Wait and pickup.
7. Back to home position.

## TODO

- [ ] Gazebo simulation: having trouble with the controller_manager
- [x] Walker sbpl planning interface config
- [x] Walker move_group.launch
- [x] Conveyor belt simulation: use fake ar_tag tracking
- [x] Import nrw_demo pipeline
- [x] Specify home positions (above the conveyor?)
- [x] Enable both arms: add left arm picks
- [ ] Change the tip link to palm link: plan the center of the palm to grasp pose
- [ ] Add grasping movements for hands
- [ ] Take care not to topple the objects that the robot is not grasping 
- [ ] Record statistics about the planner: planning time & execution time for the grasps (first & second)
- [ ] Play with the height and speed of the conveyor, home position of the right arm 

## Troubleshooting

- When running ```demo.launch```, error message ```[ERROR]: Unable to connect to move_group action server 'move_group' within allotted time (30s)``` popped out: move_group hasn't been fully launched before connecting to rviz. Fix: Wait for a moment, restart rviz. 
- Error: ```TF has no common time between '/odom_combined' and 'base_footprint'```. Run ```rosrun tf static_transform_publisher 0 0 0 0 0 0 base_footprint odom_combined 13```, assume the robot is static.
- Enabled the snaps, but the planner was not using it: snap motion primitives were not being generated because no IK solution found! Change kinematic.yaml parameters.

