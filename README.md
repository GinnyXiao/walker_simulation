# Walker_simulation

Pick-and-place simulation for wheeled walker robot.

## Getting Started

1. Launch move_group: ```roslaunch walker_simulation move_group.launch```
2. Run Rviz and use this config: ```walker_simulation.rviz```
3. Run conveyor simulator: ```rosrun walker_simulation conveyor_simulator```
4. Run demo: ```rosrun walker_simulation walker_simulation_node```

## TODO

- [ ] Gazebo simulation: having trouble with the controller_manager
- [x] Walker sbpl planning interface config
- [x] Walker move_group.launch
- [x] Conveyor belt simulation: use fake ar_tag tracking
- [x] Import nrw_demo pipeline
- [x] Specify home positions (above the conveyor?)
- [ ] Enable both arms: add left arm picks
- [ ] Change the tip link to palm link: plan the center of the palm to grasp pose
- [ ] Add grasping movements for hands
- [ ] Take care not to topple the objects that the robot is not grasping 

## Troubleshooting

- When running ```demo.launch```, error message ```[ERROR]: Unable to connect to move_group action server 'move_group' within allotted time (30s)``` popped out: move_group hasn't been fully launched before connecting to rviz. Fix: Wait for a moment, restart rviz. Or start `\roscore` before launching.
- Error: ```TF has no common time between '/odom_combined' and 'base_footprint'```. Run ```rosrun tf static_transformublisher 0 0 0 0 0 0 base_footprint odom_combined 13```, assume the robot is static.
- Enabled the snaps, but the planner was not using it: snap motion primitives were not being generated because no IK solution found! Change kinematic.yaml parameters.
