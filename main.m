%% Lab1 Challenge
leftStart = robot.encoders.LatestMessage.Data(1);
rghtStart = robot.encoders.LatestMessage.Data(2);

signedDistance = 0;

startTime = tic;
while(signedDistance<2)
    sendVelocity(robot, 0.5, 0.5)
    
    leftDistance = robot.encoders.LatestMessage.Data(1) - leftStart;
    rghtDistance = robot.encoders.LatestMessage.Data(2) - rghtStart;
    signedDistance = ((leftDistance) + (rghtDistance))/2;
    
    pause(0.05)
end
sendVelocity(robot, 0, 0)
pause(0.05)
while(signedDistance>0)
    sendVelocity(robot, -0.5, 0.5)
    
    leftDistance = robot.encoders.LatestMessage.Data(1) - leftStart;
    rghtDistance = robot.encoders.LatestMessage.Data(2) - rghtStart;
    signedDistance = ((leftDistance) + (rghtDistance))/2;
    
    pause(0.05)
end
sendVelocity(robot, 0, 0)
