robot = NohBot();
plot = DisplacementLinePlotter(0, 5, -0.5, 0);
sTime = tic;


global encoderData;
global encoderTime

prevEnc = encoderData;
prevT = encoderTime;
time0 = prevT;
while(toc(sTime) < 5)
    robot.move(0.1, 0.1);
    
    enc = encoderData;
    T = encoderTime;
    
    l = enc(1);
    r = enc(2);
    prevL = prevEnc(1);
    prevR = prevEnc(2);
    dsl = l-prevL;
    dsr = r-prevR;
    dt = T - prevT;
    
    
    vl = dsl/dt;
    vr = dsr/dt;
    
    prevEnc = enc;
    prevT = T;
    
    AddToArrays(plot, T- time0, [vl, vr]);
    pause(0.2);
end

robot.move(0, 0);