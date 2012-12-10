#!/bin/bash
rosservice call gazebo/apply_body_wrench '{body_name: "ball1::ball" , wrench: { torque: { x: 0, y: 2 , z: 0 } }, start_time: 10000000000, duration: 1000000000 }';

sleep 1;

rosservice call gazebo/apply_body_wrench '{body_name: "ball3::ball" , wrench: { torque: { x: 2, y: 0 , z: 0 } }, start_time: 10000000000, duration: 1000000000 }';

sleep 1;

# rosservice call gazebo/apply_body_wrench '{body_name: "ball3::ball" , wrench: { torque: { x: 0, y: -1 , z: 0 } }, start_time: 10000000000, duration: 1000000000 }';

# sleep 1;

# rosservice call gazebo/apply_body_wrench '{body_name: "ball1::ball" , wrench: { torque: { x: 0, y: -2 , z: 0 } }, start_time: 10000000000, duration: 1000000000 }';

# sleep 1;

# rosservice call gazebo/apply_body_wrench '{body_name: "ball2::ball" , wrench: { torque: { x: -2, y: 0 , z: 0 } }, start_time: 10000000000, duration: 1000000000 }';

# sleep 1;

# rosservice call gazebo/apply_body_wrench '{body_name: "ball3::ball" , wrench: { torque: { x: 0, y: 1 , z: 0 } }, start_time: 10000000000, duration: 1000000000 }';
