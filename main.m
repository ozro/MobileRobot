robot = NohBot();
plot = TimePlotter(0, 5, 0, 0.3);
startTime = tic;


global encoderData;
global encoderTime
global sTime;

prevEnc(1) = robot.rasp.encoders.LatestMessage.Vector.X;
prevEnc(2) = robot.rasp.encoders.LatestMessage.Vector.Y;
time0 = sTime;
prevT = time0;
while(toc(startTime) < 2)
    robot.move(-0.1, -0.1);
    
    enc = encoderData;
    T = encoderTime - time0;
    l = enc(1);
    r = enc(2);
    prevL = prevEnc(1);
    prevR = prevEnc(2);
    dsl = l-prevL;
    dsr = r-prevR;
    dt = T - prevT;
    
    vl = dsl/dt;
    vr = dsr/dt;
    
    plot = Update(plot, T, vr);
        
    prevEnc = enc;
    prevT = T;
    pause(0.1);
end

robot.move(0, 0);