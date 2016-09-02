%% Lab1 Task1 Move Robot
timerVal = tic;
while(toc(timerVal) < 5)
    sendVelocity(robot,0.01, 0.05);
    pause(0.05)
end
sendVelocity(robot,0,0);
timerVal = tic;
while(toc(timerVal) < 5)
    sendVelocity(robot,-0.05, -0.01);
    pause(0.05)
end
sendVelocity(robot,0,0);
