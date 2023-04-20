Interface haut niveau/bas niveau, point de vue STM32
~~ en barré ~~ fonctionnalité si on a le temps

| Nom             | Pub(stm->rpi)/Sub(rpi->stm) | syntaxe(old) | syntaxe (protobuf)                              | Description                                                                             |
|-----------------|-----------------------------|--------------|-------------------------------------------------|-----------------------------------------------------------------------------------------|
| nb_display      | Sub   | a d %d       | NbDisplay(int nb)                               |                                                                                         |
| odom_pos        | Pub                         | r x y theta  | Position (float x, float y, float theta)        |                                                                                         |
| robot_pos       | Sub                         | @ %f %f %f   | Position (float x, float y, float theta)        | Robot position to set to low level calculated from lidar (& odom fusion if implemented) |
| reach_pos       | Sub           | @ %f %f %f   | Position (float x, float y, float theta)        | Position to reach in direct path                                          |
| ~~reach_speed~~ | Sub           | v %f %f %f   | Speed (float vx, vy,vtheta)                    |                                                   |
| odom_speed      | Pub           | ! %f %f %f   | Speed (float vx, vy,vtheta)                    |                                                   |
| arm_state       | Pub           | a a %d       | ArmState (int nb)       | state of the pince/recup gateau system                                    |
| start           | Pub           | c TI 42      | Bool (bool value)       | if match startedboo                                                       |
| color           | Pub           |              | Bool (bool value)       | Color of the team                                                         |
| stop            | Sub           | s %d         | int                     | stop consign if obstacle (or match ended) 0=ok, 1 == slow down, 2 == stop               |
| write_ax12      | Sub        | a a%d %d     | Ax12(int id, int val, int speed)                | set ax12 position                                                      |
| turbine         | Sub        |              | Bool (bool value)                               | toggle on/off turbine                                                  |
|                 |            |              |                                                 |                                                                        |
| ~~pid~~         | sub        | g %c %d %d   | Pid (String name, float kp, float ki, float kd) |                                                                                         |