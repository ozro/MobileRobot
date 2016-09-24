robot = NohBot();
global encoderData;
global encoderTime;
global sTime;
sEnc = encoderData;

refArray = zeros(1,1);
realArray = zeros(1,1);
delArray= zeros(1,1);
timeArray = zeros(1,1);
realTime = zeros(1,1);
diffArray = zeros(1,1);

prevPe = 0;

t = encoderTime - sTime;
prevTime = t;
prevS = 0;
while(t < 7)
    t = encoderTime - sTime;
    dt = t - prevTime;
    if(dt == 0)
        pause(0.001);
        continue;
    end
    prevTime = t;

    %% Feed forward
    enc = encoderData;
    s = (enc(1) - sEnc(1) + enc(2) - sEnc(2))/2;

    uRef = trapezoidalV(t, 0.75, 0.25, 1, 1);
    uDel = trapezoidalV(t - robot.delay, 0.75, 0.25, 1, 1);
    %% Feedback PID
    kP = 3;
    kD = 0.15;
    
    Pe = delArray(end) + uDel*dt - s;
    prevS = s;

    De = (Pe - prevPe)/dt;
    prevPe = Pe;
    
    eV = kP * Pe + kD * De;
    
    V = uRef+eV;
    if(V ~= 0)
        sgn = V/abs(V);
        V = min(abs(V), 0.3);
        V = V * sgn;
    end

    robot.move(V,V);
    
    timeArray = [timeArray, t];
    realArray = [realArray, s];
    refArray = [refArray, refArray(end) + uRef * dt];
    delArray = [delArray, delArray(end) + uDel * dt];
    diffArray = [diffArray, delArray(end) - realArray(end)];
    
    figure(2)
    diffPlot = plot(timeArray, diffArray);    
    figure(1)
    plot(timeArray, refArray, 'k', timeArray, delArray, 'b', timeArray, realArray, 'r');
    
end

pause(0.001);
t = tic;
while(toc(t) < 2)
    robot.move(0,0);
    pause(0.05);
end

% robot = NohBot();
% 
% kP = 10.0;
% kD = 0.75;
% kI = 0.0;
% 
% global encoderData;
% global encoderTime;
% global sTime;
% 
% sEnc = encoderData;
% 
% pTime = sTime;
% 
% Gs = 0.1;
% s = 0;
% vMax = 0.3;
% 
% prevPe = Gs-s;
% Pe = Gs - s;
% 
% pePlot = plot(encoderTime - sTime, Pe);
% 
% while((abs(Pe) > 0.0001) && (encoderTime - sTime < 4))    
%     
%     dt = encoderTime - pTime;
%     if(dt == 0)
%         pause(0.001);
%         continue;
%     end
%     pTime = encoderTime;
%     
%     De = (Pe - prevPe)/dt
%     prevPe = Pe;
%     
%     enc = encoderData;
%     s = (enc(1) - sEnc(1) + enc(2) - sEnc(2))/2;
%     
%     Pe = Gs - s;
%     
%     v = kP * Pe + kD * De;
%     sign = v/abs(v);
%     
%     v = min(abs(v), vMax);
%     v = sign * v;
%     
%     robot.move(v, v);
%     pause (0.005);
%     
%     set(pePlot, 'xdata', [get(pePlot, 'xdata') encoderTime - sTime], 'ydata', [get(pePlot, 'ydata') Pe]);
%     pause(0.005);
% end
% 
% pause(0.001);
% t = tic;
% 
% while(toc(t) < 2)
%     robot.move(0,0);
%     pause(0.05);
% end