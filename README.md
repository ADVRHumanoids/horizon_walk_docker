# horizon_walk_docker

## Run simulator
1) open mujoco and load talos:
   ```
   roslaunch talos_horizon talos_simulator.launch
   ```
3) load xbot config file:
   ```
   roscd talos_cartesio_config/mujoco/xbot2
   set_xbot2_config talos.yaml
   ```
4) start xbot2:
   ```
   xbot2-core
   ```
5) move the robot in the homig configuration and set the sim time as the default one:
   ```
   rosservice call /xbotcore/homing/switch "data: true"
   rosparam set /use_sim_time true
6) start the controller:
   ```
   roslaunch talos_horizon talos_controller xbot:=true
   ```
   *NB:* if you don't have a joystick plugged you can run the controller with the argument `joy:=false` and move the robot around using the ros services/topics:
   ```
   # set velocity reference
   rostopic pub /horizon/base_velocity/reference geometry_msgs/Twist "linear:
      x: 0.0
      y: 0.0
      z: 0.0
    angular:
      x: 0.0
      y: 0.0
      z: 0.0"

   # start/stop walking
   rosservice call /horizon/walk/switch "data: true/false"
   ```

## Run on the robot
After initialized the robot and set up the ros communication with the ros master running on the robot, simply run the controller:
```
roslaunch talos_horizon talos_controller.launch talos:=true
```
Follow steps above if you do not have a joystick plugged
