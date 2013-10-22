#!/bin/bash
roslaunch launch/load_one_ball.launch
sleep 4;
./bin/fps_measurer &
echo STARTED
rosservice call gazebo/apply_body_wrench '{body_name: "ball_model::ball" , wrench: { torque: { x: 0, y: 2 , z: 0 } }, start_time: 10000000000, duration: 1000000000 }';

sleep 4;

rosservice call gazebo/apply_body_wrench '{body_name: "ball_model::ball" , wrench: { torque: { x: 2, y: -2 , z: 0 } }, start_time: 10000000000, duration: 1000000000 }';

sleep 4;

rosservice call gazebo/apply_body_wrench '{body_name: "ball_model::ball" , wrench: { torque: { x: -2, y: -2 , z: 0 } }, start_time: 10000000000, duration: 1000000000 }';

sleep 4;

rosservice call gazebo/apply_body_wrench '{body_name: "ball_model::ball" , wrench: { torque: { x: -2, y: 2 , z: 0 } }, start_time: 10000000000, duration: 1000000000 }';

sleep 4;
rosnode kill fps_measurer
echo ENDED
