%% Lab1 Task2 Basic Simulation
plot = DisplacementLinePlotter();

leftStart = 6934;
rghtStart = 4396;
leftVelocity = 50;
rghtVelocity = 50;

leftEncoder = leftStart;
rghtEncoder = rghtStart;

startTime = tic;
timerVal = tic;
while(toc(startTime)<5)
    loopTime = toc(timerVal);
    
    leftEncoder = leftEncoder + loopTime * leftVelocity;
    rghtEncoder = rghtEncoder + loopTime * rghtVelocity;
    
    AddToArrays(plot, toc(startTime), leftEncoder - leftStart, rghtEncoder - rghtStart);
    PlotArrays(plot);
    
    timerVal = tic;
    pause(0.05)
end