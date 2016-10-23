%robot = NohBot();
system = mrplSystem(robot,false, false);
system.executeTrajectoryToRelativePose(0.25,0.25,0,1);
pause(3)
system.executeTrajectoryToRelativePose(-0.5,-0.5,-pi()/2,1);
pause(3);
system.executeTrajectoryToRelativePose(-0.25,0.25,pi()/2,1);

% ref = RefCon(3,1,0.5);
% tf = ref.getTrajDuration() + 1;
% traj = RobotTrajectory(0, tf, ref);
% contrl = controller(robot, ref, traj, true);
% contrl.run();