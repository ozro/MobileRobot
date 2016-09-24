robot = NohBot();
global encoderData;
global encoderTime;
global sTime;

refArray = 0;
realArray = 0;
delArray=0;

prevTime = sTime;
prevEnc = encoderData;
prevV = 0;
prevU = 0;

start = tic;
prevToc = toc(start);
refTimeArray = 0;
realTimeArray = 0;
diffArray = 0.01;

Pe = 0.0011;
prevPe = 0;

while(encoderTime - sTime< 7)
    dt = encoderTime - prevTime;
    if(dt == 0)
        pause(0.001);
        continue;
    end
    prevTime = encoderTime;

    %% Feed forward
    enc = encoderData;
    realS = (enc(1) - prevEnc(1) + enc(2) - prevEnc(2))/2;
    prevEnc = enc;

    realTimeArray = [realTimeArray, encoderTime - sTime];
    realArray = [realArray, realArray(end) + realS];
    refTimeArray = [refTimeArray, encoderTime - sTime];
    refArray = [refArray, refArray(end) + prevV * dt];
    delArray = [delArray, delArray(end) + prevU * dt];
    diffArray = [diffArray, delArray(end) - realArray(end)];
    
    figure(2)
    diffPlot = plot(realTimeArray, diffArray);
    
    figure(1)
    plot(refTimeArray, refArray, 'k', refTimeArray, delArray, 'b', realTimeArray, realArray, 'r');
    
    uRef = trapezoidalV(encoderTime - sTime, 0.75, 0.25, 1, 1);
    uDel = trapezoidalV(encoderTime - sTime - robot.delay, 0.75, 0.25, 1, 1);
    prevV = uRef;
    prevU = uDel;
    
    %% Feedback PID
    kP = 0;
    kD = 0;
    
    Pe = diffArray(end);
    De = (Pe - prevPe)/dt;
    prevPe = Pe;
    
    eV = kP * Pe + kD * De;
    
    V = uRef+eV;
    sgn = V/abs(V);
    V = min(abs(V), 0.3);
    V = V * sgn;

    robot.move(V,V);
    pause(0.005)
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