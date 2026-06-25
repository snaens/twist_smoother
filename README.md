# Twist limiter

Clamps values on `twist_limiter/in` to the maximum m/s² defined in the parameter `acceleration_limit_mps2`,  
and publishes the result to `twist_limiter/out`.  
Also features a 2-second timeout, upon which it decelerates down to 0.

## Use case
Say you have a gamepad controller, and want to teleop a robot.  
You push the controller forwards, the robot leaps into action, accelerates way too fast, and breaks something.  
Fun!  

This node clamps the maximum acceleration you can give a robot to a preset value.  
Now you can teleop the robot much like videogame controls, and other nodes no longer need a soft-start.  
Neato!

## Parameters
`acceleration_limit_mps2`, default 1.5: maximum acceleration to clamp values to  
`publisher_hz` default 20: how many hertz to run the publisher at  
`maximum.linear.x, maximum.linear.y, maximum.linear.z` default 1.0, maximum values to publish and internally accumulate  
`maximum.angular.x, maximum.angular.y, maximum.angular.z` default 1.0, maximum values to publish and internally accumulate


## Tips
- remap the topics to something sensible: `ros2 run twist_limiter twist_limiter_node --ros-args -r twist_limiter/out:=/georg/cmd_vel`
- reconfigure values: `ros2 run twist_limiter twist_limiter_node --ros-args -p publisher_hz:=42`

---

Happy driving!
