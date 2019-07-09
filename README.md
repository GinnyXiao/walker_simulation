# Walker_simulation

Pick-and-place simulation for wheeled walker robot.

## Getting Started

3. Launch move_group: ```roslaunch walker_moveit_config move_group.launch```
2. Run joint_state_publisher and robot_state_publisher
3. Run Rviz and use this config: ```walker_simulation.rviz```
4. Run conveyor simulator: ```rosrun walker_simulation conveyor_simulator```
5. Run demo: ```rosrun walker_simulation walker_simulation_node```

## TODO 

- [ ] Gazebo simulation: having trouble with the controller_manager
- [ ] Walker sbpl planning interface config
- [x] Walker move_group?
- [x] Conveyor belt simulation: use ar_tag tracking
- [ ] Implement pick-and-place for walker

## Troubleshooting

