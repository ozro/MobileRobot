%Lab 11
%clear all;
robot = NohBot();
pause(0.2);
robot.laserOn();
robot.forkDown();

startPose = [0.6096, 0.6096, pi/2];

image = rangeImage();
while(true)
    est = stateEstimator(pose(startPose(1), startPose(2), startPose(3)), robot);
    system = mrplSystem(robot,est, true, true, startPose);

    approachSail(system, image,est, robot.offset);
    approachSail(system,image, est, robot.offset - 10/100);
    robot.laserOff();
    
    pause(0.1);
    system.moveRel(0.06);
    pause(0.1);
    robot.forkUp();
    pause(1);
    robot.forkDown();
    pause(1);
    system.moveRel(-0.06);
    pause(0.5);
    system.turnTh(pi);
    startPose = est.fusePose.getPoseVec()';
    beep;
    pause(7);
    beep;
    robot.laserOn();
    
end

