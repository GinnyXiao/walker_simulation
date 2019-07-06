# Walker_simulation

Pick-and-place simulation for wheeled walker robot.

## Getting Started

1. Launch URDF and robot_state_publisher: ```roslaunch walker_planner planning_context_walker.launch```
2. Launch planner: ```roslaunch walker_planner walker_interface_planner.launch```
3. Launch move_group: 
4. Run Rviz and use this config: ```src/wheeled_walker/urdf.rviz```

## TODO 

- Gazebo simulation: having trouble with the controller_manager
- Walker move_group?
- Conveyor belt simulation: use ar_tag tracking
- Implement pick-and-place for walker

## Troubleshooting

