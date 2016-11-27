robot = NohBot();
pause(0.2);
robot.laserOn();
robot.forkDown();

startPose = [0.3048, 0.3048, -pi/2];
targetPose = [0.3048 * 2, 4*0.3048, pi/2];
dropPose = [0.3048 * 2.5, 0.3048, -pi/2];

image = rangeImage();
est = stateEstimator(pose(startPose(1), startPose(2), startPose(3)), robot);
system = mrplSystem(robot,est, true, false, startPose);

system.turnTh(pi);
system.executeTrajectoryToAbsPose(targetPose(1), targetPose(2) - 30/100, targetPose(3), 1, 0.25);

%approachSail(system, image,est, robot.offset + 10/100);
approachSail(system,image, est, robot.offset);
robot.laserOff();

pause(0.1);
system.moveRel(0.06);
pause(0.1);
robot.forkUp();
pause(0.1);
system.moveRel(-0.06);
pause(0.1);
system.turnTh(pi);
startPose = est.fusePose.getPoseVec()';
est = stateEstimator(pose(startPose(1), startPose(2), startPose(3)), robot);
system = mrplSystem(robot,est, true, false, startPose);
system.executeTrajectoryToAbsPose(dropPose(1), dropPose(2), dropPose(3), 1, 0.25);

% robot = NohBot();
% pause(0.2)
% robot.forkDown();
% startPose = [0.3048, 0.3048, -pi/2];
%     
% est = stateEstimator(pose(startPose(1), startPose(2), startPose(3)), robot);
% system = mrplSystem(robot,est, true, false, startPose);
% 
% targetPose = [0.3048 * 2, 4*0.3048, pi/2];
% dropoffPose = [0.3048 * 2.5, 0.3048, pi/2];
% 
% system.blind(startPose, targetPose, 0.25);
% 
% robot.laserOn();
% image = rangeImage();
% startPose = [targetPose(1), targetPose(2) - 35/100, -startPose(3)];
% est = stateEstimator(pose(startPose(1), startPose(2), startPose(3)), robot);
% system = mrplSystem(robot,est, true, false, startPose);
% approachSail(system, image, est, robot.offset);
% robot.laserOff();
% 
% pause(0.1);
% system.moveRel(0.06, 0.3);
% pause(0.2);
% robot.forkUp();
% system.moveRel(-0.06, 0.3);
% pause(0.2);
% 
% system.blind(targetPose, dropoffPose, 0.25);
% %system.executeTrajectoryToAbsPose(dropoffPose(1),dropoffPose(2),dropoffPose(3),1,0.1);
% 
% 
% robot.forkDown();