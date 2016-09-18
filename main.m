robot = NohBot();

v = 0.2;
ks = 3;
kk = 15.1084/ks;
sf = 1*ks;
kth = 2*pi/sf;
tf = sf/v;

x = 0;
y = 0;
th = 0;

% xP = zeros(1,1);
% yP = zeros(1,1);
% comP = plot(xP, yP);
% xlim([-0.6, 0.6]);
% ylim([-0.6, 0.6]);

xR = zeros(1,1);
yR = zeros(1,1);
realP = plot(xR, yR);
xlim([-0.6, 0.6]);
ylim([-0.6, 0.6]);

global encoderData;
global encoderTime;
global sTime;

t = encoderTime - sTime;
prevT = t;

vl = 0;
vr = 0;

rx = 0;
ry = 0;
rth = 0;

prevEnc = encoderData;
while(t < tf)
    t = encoderTime - sTime;
    if(t > tf)
        break;
    end
    dt = t - prevT;
    if(dt == 0)
        pause(0.001)
        continue;
    end
    prevT = t;
    
    s = v*t;
    k = kk * sin(kth*s);
    omg = k * v;
    
    [vl, vr] = robot.angVelToWheel(v, omg);
    robot.move(vl, vr);
    
    th = th + omg*dt/2;
    x = x+ v * cos(th) * dt;
    y = y + v * sin(th) * dt;
    th = th + omg * dt/2;
    %set(comP, 'xdata', [get(comP, 'xdata') x], 'ydata', [get(comP, 'ydata') y]);
    pause(0.0025);
    
    enc = encoderData;
        
    vl = (enc(1) - prevEnc(1))/dt;
    vr = (enc(2) - prevEnc(2))/dt;
    angVel = robot.wheelToAngVel(vl, vr);
    rv = (vl + vr) /2;
    
    rth = rth + angVel * dt / 2;
    rx = rx + rv * cos(rth) * dt;
    ry = ry + rv * sin(rth) * dt;
    rth = rth + angVel * dt / 2;
    
    prevEnc = enc;
        
    set(realP, 'xdata', [get(realP, 'xdata') rx], 'ydata', [get(realP, 'ydata') ry]);

    pause(0.005);
end
pause(0.005);
robot.move(0,0);
 
% t = 0;
% dt = 0.1;
% v = 0.1;
% th = 0;
% x = 0;
% y = 0;

% while(t<sqrt(32*pi))
%     t = t + dt;
%     omg = 1/8*t;
%     th = th + omg*dt/2;
%     x = x + v*cos(th)*dt;
%     y = y + v*sin(th)*dt;
%     th = th + omg*dt/2;
%     pause(0.1);
%     plot = Update(plot, x, y);
%     pause(0.1);
% end


%robot = NohBot();
% plot = TimePlotter(0, 5, 0, 0.3);
% startTime = tic;
% 
% 
% global encoderData;
% global encoderTime
% global sTime;
% 
% prevEnc(1) = robot.rasp.encoders.LatestMessage.Vector.X;
% prevEnc(2) = robot.rasp.encoders.LatestMessage.Vector.Y;
% time0 = sTime;
% prevT = time0;
% while(toc(startTime) < 2)
%     robot.move(-0.1, -0.1);
%     
%     enc = encoderData;
%     T = encoderTime - time0;
%     l = enc(1);
%     r = enc(2);
%     prevL = prevEnc(1);
%     prevR = prevEnc(2);
%     dsl = l-prevL;
%     dsr = r-prevR;
%     dt = T - prevT;
%     
%     vl = dsl/dt;
%     vr = dsr/dt;
%     
%     plot = Update(plot, T, vr);
%         
%     prevEnc = enc;
%     prevT = T;
%     pause(0.1);
% end
% 
% robot.move(0, 0);