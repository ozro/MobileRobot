%robot = NohBot();
robot.laserOn();
image = rangeImage();
system = mrplSystem(robot,true, false);

global laserRanges;

ranges = laserRanges;
[xbar, ybar, th, xArray, yArray, allX, allY] = image.findLineCandidate(ranges);

plot(-1*allY, allX, '.b', -1*yArray, xArray, '.r', 0, 0, 'ok');
title('Laser Range Positions');
legend('Other Points', 'Sail');
xlabel('y (m)');
ylabel('x (m)');

xlim([-4, 4])
ylim([-4, 4])

robot.laserOff();
pause(0);

sgn = 1;
if(xbar < 0 && abs(ybar) < 1)
    sgn = -1;
end

th
xbar = xbar - (robot.length + robot.palletL)*cos(th)
ybar = ybar - (robot.length + robot.palletL)*sin(th)
system.executeTrajectoryToRelativePose(xbar,ybar,th,sgn);