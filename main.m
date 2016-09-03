%% Lab1 Challenge
plot = DisplacementLinePlotter();

leftStart = robot.encoders.LatestMessage.Data(1);
rghtStart = robot.encoders.LatestMessage.Data(2);

signedDistance = 0;

startTime = tic;
while(signedDistance<200)
    pause(0.1)
    sendVelocity(robot, 0.05, 0.05)

    leftDistance = robot.encoders.LatestMessage.Data(1) - leftStart;
    rghtDistance = robot.encoders.LatestMessage.Data(2) - rghtStart;
    signedDistance = ((leftDistance) + (rghtDistance))/2;
    
    AddToArrays(plot, toc(startTime), leftDistance, rghtDistance);
    pause(0.05)
end
pause(0.1)
sendVelocity(robot, 0, 0)
pause(2)

while(signedDistance> -104.8)
    pause(0.1)
    sendVelocity(robot, -0.05, -0.05)
    
    leftDistance = robot.encoders.LatestMessage.Data(1) - leftStart;
    rghtDistance = robot.encoders.LatestMessage.Data(2) - rghtStart;
    signedDistance = ((leftDistance) + (rghtDistance))/2;
    
    AddToArrays(plot, toc(startTime), leftDistance, rghtDistance);
    pause(0.05)
end
pause(0.1)
sendVelocity(robot, 0, 0)
