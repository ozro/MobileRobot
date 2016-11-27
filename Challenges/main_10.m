%Lab 10
robot = NohBot();
robot.laserOn();
est = stateEstimator(pose(0.6096, 0.6096 - 0.045, pi/2), robot);
system = mrplSystem(robot,est,true,true, [0.6096, 0.6096 - 0.045, pi/2]);
system.executeTrajectoryToAbsPose(0.3048, 0.9144 - 0.025, pi/2, 1, 0.25);
pause(20);
system.executeTrajectoryToAbsPose(0.9144 + 0.0125,0.3048 + 0.045, pi/15, 1, 0.25);
pause(20);
system.executeTrajectoryToAbsPose(0.6090 - 0.02, 0.6096 - 0.045, pi/2, 1, 0.25);
shut


% robot = NohBot();
% robot.laserOn();
% h = figure;
% KPDriver(h);
% vGain = 5;
% 
% p1 = [0,      0; 
%       0, 1.2192];
% p2 = [1.2192, 0; 
%       0, 0];
% LML = LineMapLocalizer(p1, p2,0.01, 0.001, 0.0005);
% robotPose = pose(24*0.0254, 24*0.0254, pi/2);
% est = stateEstimator(robotPose, robot);
% 
% while(true)
%     est.processOdometryData();
%     est.processRangeImage();
%     KPDriver.drive(robot, vGain);
%     fusePose = est.fusePose.getPoseVec();
%     odoPose = est.odoPose.getPoseVec();
%     plot(fusePose(1), fusePose(2), 'ob', odoPose(1), odoPose(2), 'r*');
%     xlim([0, 1.5]);
%     ylim([0, 1.5]);
%     title('Fuse Pose');
%     pause(0.05);
% end
% 
