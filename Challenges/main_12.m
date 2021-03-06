robot = NohBot();
figure(1);
figure(2);
figure(3);
pause(0.2);
robot.laserOn();
robot.forkDown();

startPose = [0.75*0.3048,0.75*0.3048, -pi/2];

target = [1*0.3048, 3.5*0.3048, pi/2;
          2*0.3048, 3.5*0.3048, pi/2;
          3*0.3048, 3.5*0.3048, pi/2];
drop = [1.75*0.3048, 0.75*0.3048, -pi/2;
        2.25*0.3048, 0.75*0.3048, -pi/2;
        2.75*0.3048, 0.75*0.3048, -pi/2];

image = rangeImage();
est = stateEstimator(pose(startPose(1), startPose(2), startPose(3)), robot);
system = mrplSystem(robot,est, true, true, startPose);
for i = 1:3
    targetPose = target(i,:);
    dropPose = drop(i,:);
    pause(0.2)
    system.turnTh(pi)
    wait(est,1.5);

    system.refPoseW = est.fusePose.getPoseVec()';
    system.executeTrajectoryToAbsPose(targetPose(1), targetPose(2) - 35/100, targetPose(3), 1, 0.15, true);
    wait(est,1.5);
    system.refPoseW = est.fusePose.getPoseVec()';
    approachSail(system,image, est, robot.offset);
    
    pause(0.1);
    system.moveRel(0.06);
    pause(0.5);
    robot.forkUp();
    pause(0.1);
    system.moveRel(-0.06);
    pause(0.5);
    system.turnTh(pi);
    pause(0.5);
    system.moveRel(0.05);
    wait(est,2);
% 
%     targetPose(3) = -targetPose(3);
%     targetPose(2) = targetPose(2) - robot.offset/2;
%     system.refPoseW = targetPose;
    system.refPoseW = est.fusePose.getPoseVec()';
    system.executeTrajectoryToAbsPose(dropPose(1), dropPose(2) + robot.offset - 0.06, dropPose(3), 1, 0.05, true);
    
    pause(0.1);
    robot.forkDown();
    pause(0.1);
    system.moveRel(-0.1);
    pause(0.5);
end
robot.move(0,0);
robot.laserOff();

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