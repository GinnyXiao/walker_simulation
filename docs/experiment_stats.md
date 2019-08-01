# Statistics

- planning group: right arm
- home configuration (joint angles): [ -0.736, -1.052, 0.243, -0.807, 0.2405, 0.017, 0.133 ]
- drop-off configuration: [ -0.0345, -1.4979, 0.1055, -0.7480, 0.2405, 0.017, 0.133 ]
- planner id: arastar.bfs.manip
- SNAP_TO_XYZ_RPY thresh: 0.200
- allowed planning time: 3.0 secs
- conveyor speed: 0.1 m/s
- demo pipeline: home config ==> predicted goal pose ==> drop-off config ==> home config

|                  | Planning time | Total time (plan + execute) |
| :--------------- | :-----------: | :-------------------------: |
| home -> goal     |     0.39      |            3.85             |
| goal -> drop-off |     1.10      |            6.15             |
| drop-off -> home |     0.16      |            2.78             |

|                  | Planning time | Total time (plan + execute) |
| :--------------- | :------------ | :-------------------------- |
| home -> goal     |               |                             |
| goal -> drop-off |               |                             |
| drop-off -> home |               |                             |