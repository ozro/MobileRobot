%Lab 7
robot = NohBot();
robot.laserOn();
system = mrplSystem(robot,true,true, [0.6096, 0.6096, pi/2]);
system.executeTrajectoryToAbsPose(0.3048, 0.9144, pi/2, 1);
pause(3);
system.executeTrajectoryToAbsPose(0.9144,0.3048,0, 1);
pause(3);
system.executeTrajectoryToAbsPose(0.6090, 0.6096, pi/2, 1);
shut
