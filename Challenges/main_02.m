% Lab02
%% Initialization
robot = NohBot();
robot.laserOn();
targetPlot = PointPlotter(-2, 2, -2, 2);

%% State variables
prevTime = tic;
prevDist = 1.6;
prevBearing = 0;
hasTarget = true;

%% Parameters
detectDist = 1.5;
followDist = 0.5;
followSpeed = 0.15;

%% Main loop
while(true)
    ranges = robot.laserRanges();
    
    targetDist = detectDist + 1;
    targetBearing = 0;
    for i = [1:91,270:360]
        if (ranges(i)<=1.5 && ranges(i)>=0.06 && ranges(i)<targetDist )
            targetDist = ranges(i);
            targetBearing = i-1;
        end
    end
    
    hasTarget = true;
    if(targetDist ~= 1.6)
        prevTime = tic;
        prevDist = targetDist;
        prevBearing = targetBearing;
    elseif(toc(prevTime) <= 0.75)
        targetDist = prevDist;
        targetBearing = prevBearing;
        
    else
        hasTarget = false;
        UpdatePoints(targetPlot,0,0);
        robot.move(0,0);
    end
    
    if hasTarget
        [x,y] = Calc.irToxy(targetBearing, targetDist);
        UpdatePoints(targetPlot,x,y);

        if(targetDist < followDist)
            robot.move(-followSpeed, -followSpeed);

        elseif(abs(targetDist-followDist) <= 0.05)
            robot.move(0,0);

        else
            angVel = Calc.getAngVel(followSpeed,targetBearing,targetDist);
            if (targetBearing>90) %right side of robot
                angVel = angVel * (-1);
            end
            
            [vl, vr] = robot.angVelToWheel(angVel);
            robot.move(vl, vr);
        end
    end
    pause(0.2)
end