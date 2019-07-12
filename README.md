# Walker_simulation

Pick-and-place simulation for wheeled walker robot.

## Getting Started

1. Launch move_group: ```roslaunch walker_moveit_config move_group.launch```
2. Run Rviz and use this config: ```walker_simulation.rviz```
3. Run conveyor simulator: ```rosrun walker_simulation conveyor_simulator```
4. Run demo: ```rosrun walker_simulation walker_simulation_node```

## TODO 

- [ ] Gazebo simulation: having trouble with the controller_manager
- [x] Walker sbpl planning interface config
- [x] Walker move_group.launch
- [x] Conveyor belt simulation: use ar_tag tracking
- [ ] Implement pick-and-place for walker

## Troubleshooting

- When running ```demo.launch```, error message ```[ERROR]: Unable to connect to move_group action server 'move_group' within allotted time (30s)``` popped out: move_group hasn't been fully launched before connecting to rviz. Fix: Wait for a moment, restart rviz. 
- Error: ```TF has no common time between '/odom_combined' and 'base_footprint'```. Run ```rosrun tf static_transformublisher 0 0 0 0 0 0 base_footprint odom_combined 13```, assume the robot is static.