robot = NohBot();
figure(1);
figure(2);
%figure(3);
pause(0.2);
robot.laserOn();
robot.forkDown();

startPose = [0.75*0.3048,0.75*0.3048, -pi/2];

target = [1*0.3048 - 4.5/100, 6*0.3048, pi/2;
          2*0.3048 - 10/100, 6*0.3048, pi/2;
          3*0.3048 - 20/100, 6*0.3048, pi/2;
          4*0.3048 - 30/100, 6*0.3048, pi/2;
          5*0.3048 - 40/100, 6*0.3048, pi/2;
          6*0.3048 - 50/100, 6*0.3048, pi/2;
          6*0.3048, 1.5*0.3048,    0;
          
          7*0.3048 - 50/100, 6*0.3048, pi/2;
          6*0.3048, 2*0.3048,    0;
          6*0.3048, 4*0.3048,    0;];
drop = [1*0.3048, 1*0.3048, -pi/2;
        2*0.3048, 1*0.3048, -pi/2;
        3*0.3048, 1*0.3048, -pi/2;
        4*0.3048, 1*0.3048, -pi/2;
        5*0.3048, 1*0.3048, -pi/2;
        6*0.3048, 1*0.3048, -pi/2;
        5*0.3048, 1*0.3048, -pi/2];
    
image = rangeImage();
est = stateEstimator(pose(startPose(1), startPose(2), startPose(3)), robot);
system = mrplSystem(robot,est, true, true, startPose);
for i = 1:7
    targetPose = target(i,:);
    dropPose = drop(i,:);
    pause(0.2)
    system.turnTh(pi)
    wait(est,1.5);

    system.refPoseW = est.fusePose.getPoseVec()';
    
    if(i>=7)
        system.executeTrajectoryToAbsPose(targetPose(1) - 40/100, targetPose(2), targetPose(3), 1, 0.3, true);
    else
        system.executeTrajectoryToAbsPose(targetPose(1), targetPose(2) - 40/100, targetPose(3), 1, 0.3, true);
    end
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
    if(i<7)
        system.turnTh(pi);
        pause(0.5);
    end
    wait(est,1);
    
    system.refPoseW = est.fusePose.getPoseVec()';
    if(i<7)
        system.executeTrajectoryToAbsPose(system.refPoseW(1), dropPose(2) + robot.offset - 0.06, dropPose(3), 1, 0.20, true);
    else
        system.executeTrajectoryToAbsPose(dropPose(1), dropPose(2) + robot.offset - 0.06, dropPose(3), -1, 0.20, true);
    end
    pause(0.1);
    robot.forkDown();
    pause(0.1);
    system.moveRel(-0.1);
    pause(0.5);
end
robot.move(0,0);
robot.laserOff();