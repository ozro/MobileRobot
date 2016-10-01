ref = RefCon(3,1,0.5);
tf = ref.getTrajDuration() + 1;
traj = RobotTrajectory(0, tf, ref);
robot = NohBot();

t = 0;
prevt = 0;
sTime = tic;
realPose = zeros(3,1);

global encoderData;
global encoderTime;

prevEnc = encoderData;
enc = encoderData;
prevT = encoderTime;
T = encoderTime;

x = 0;
y = 0;
b = 0;
v = 0;
w = 0;
prevV = 0;
prevW = 0;
while(t<tf)
    if(t == 0)
        t = toc(sTime);
        continue;
    end
    t = toc(sTime);
    dt = t- prevt;
    prevt = t;
    
    T = encoderTime;
    enc = encoderData;
    if(T == prevT)
        pause(0.0001)
        continue;
    end
    dT = T - prevT;
    prevT = T;
    
    vl = (enc(1) - prevEnc(1))/ dT;
    vr = (enc(2) - prevEnc(2))/ dT;
    
    [v,w] = robot.wheelToAngVel(vl,vr);
    
    v1 = (prevV + v) / 2;
    w1 = (prevW + w) / 2;
    
    b = b + w1 * dt/2;
    x = x + v * cos(b) * dt;
    y = y + v * sin(b) * dt;
    b = b + w1 * dt/2;
    
    prevV = v;
    prevW = w;
    
    realPoseW = [x, y, b];
    refPoseW = traj.getPose(t);
    
    ePoseW = refPoseW - realPoseW;
    
end


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