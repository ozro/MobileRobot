%Lab 7
robot = NohBot();
system = mrplSystem(robot,true,true, [0, 0, pi/2]);
system.executeTrajectoryToAbsPose(0.25, 0.25, 0,1);
pause(3);
system.executeTrajectoryToAbsPose(-0.25,-0.25,-pi()/2,1);
pause(3);
system.executeTrajectoryToAbsPose(0, 0, 0,1);
