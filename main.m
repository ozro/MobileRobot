%% Lab1 Challenge
plot = DisplacementLinePlotter();

signedDistance = 0;

sendVelocity(robot, 0.0, 0.0);
startTime = tic;

leftStart = robot.encoders.LatestMessage.Data(1);
rghtStart = robot.encoders.LatestMessage.Data(2);

while(signedDistance < 187.5)
    pause(0.1);
    sendVelocity(robot, 0.05, 0.05);
    
    leftDistance = robot.encoders.LatestMessage.Data(1) - leftStart;
    rghtDistance = robot.encoders.LatestMessage.Data(2) - rghtStart;
    signedDistance = ((leftDistance) + (rghtDistance))/2;
    
    AddToArrays(plot, toc(startTime), leftDistance, rghtDistance);
end
pause(0.13);
sendVelocity(robot, 0.0, 0.0);
pause(2.0);

lStart2 = robot.encoders.LatestMessage.Data(1);
rStart2 = robot.encoders.LatestMessage.Data(2);
signedDis2 = 0.0;

while(signedDis2 > -281.8)
    pause(0.12);
    sendVelocity(robot, -0.05, -0.05);
    
    leftPos = robot.encoders.LatestMessage.Data(1);
    rghtPos = robot.encoders.LatestMessage.Data(2);
    signedDis2 = ((leftPos-lStart2) + (rghtPos-rStart2))/2;
    
    AddToArrays(plot, toc(startTime), leftPos-leftStart, rghtPos-rghtStart);
end
pause(0.15);
sendVelocity(robot, 0.0, 0.0);
