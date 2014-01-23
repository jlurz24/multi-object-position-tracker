#!/bin/bash
roslaunch launch/load_five_balls.launch
./bin/fps_measurer &
./bin/measurement_controller start
echo Starting movement script
echo Executing Move 1
rosservice call gazebo/apply_body_wrench '{body_name: "ball1::ball" , wrench: { torque: { x: 0, y: 2 , z: 0 } }, start_time: 10000000000, duration: 1000000000 }';

rosservice call gazebo/apply_body_wrench '{body_name: "ball2::ball" , wrench: { torque: { x: 0, y: -2 , z: 0 } }, start_time: 10000000000, duration: 1000000000 }';

sleep 2;
echo Executing Move 2
rosservice call gazebo/apply_body_wrench '{body_name: "ball1::ball" , wrench: { torque: { x: 2, y: -2 , z: 0 } }, start_time: 10000000000, duration: 1000000000 }';

rosservice call gazebo/apply_body_wrench '{body_name: "ball2::ball" , wrench: { torque: { x: -1.5, y: 2 , z: 0 } }, start_time: 10000000000, duration: 1000000000 }';

sleep 4;
echo Executing Move 3
rosservice call gazebo/apply_body_wrench '{body_name: "ball1::ball" , wrench: { torque: { x: -2, y: -2 , z: 0 } }, start_time: 10000000000, duration: 1000000000 }';

rosservice call gazebo/apply_body_wrench '{body_name: "ball2::ball" , wrench: { torque: { x: 2, y: 2 , z: 0 } }, start_time: 10000000000, duration: 1000000000 }';

sleep 4;
echo Executing Move 4
rosservice call gazebo/apply_body_wrench '{body_name: "ball1::ball" , wrench: { torque: { x: -2, y: 2 , z: 0 } }, start_time: 10000000000, duration: 1000000000 }';

rosservice call gazebo/apply_body_wrench '{body_name: "ball2::ball" , wrench: { torque: { x: 2, y: -1 , z: 0 } }, start_time: 10000000000, duration: 1000000000 }';

sleep 4;
echo Movement script completed
rosnode kill fps_measurer
./bin/measurement_controller stop
