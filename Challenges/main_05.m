robot = NohBot();
ref = RefCon(3,1,0.5);
tf = ref.getTrajDuration() + 1;
traj = RobotTrajectory(0, tf, ref);
contrl = controller(robot, ref, traj, true);
contrl.run();


% ref = RefCon(3,1,0.5);
% tf = ref.getTrajDuration();
% traj = RobotTrajectory(0, tf+1, ref);
% t=0.2235403;
% dt = 0.0028394;
% n = int32((tf)/dt);
% pose = zeros(n,3);
% index = 1;
% while(t<tf)
%     pose(index,:) = traj.getPose(t);
%     index = index +1;
%     t = t+dt;
% end
% 
% plot(pose(:,1), pose(:,2));

% robot = NohBot();
% ref = RefCon(2, 1, 0.5);
% 
% sTime = tic;
% while(true)
%     [vel, angvel] = ref.computeControl(toc(sTime));
%     [vl, vr] = robot.angVelToWheel(vel, angvel);
%     robot.move(vl, vr);
% end