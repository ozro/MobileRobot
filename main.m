clear all;
robot = NohBot();
robot.laserOn();
image = rangeImage();
system = mrplSystem(robot,true, false);

global laserRanges;

ranges = laserRanges;
[xbar, ybar, th, xArray, yArray, allX, allY] = image.findLineCandidate(ranges);

plot(0, 0, 'ok',-1*allY, allX, '.b', -1*yArray, xArray, '.r');
title('Robot Frame Laser Range Positions');
legend('Robot', 'Other Points', 'Sail');
xlabel('y (m)');
ylabel('x (m)');

xlim([-4, 4])
ylim([-4, 4])

robot.laserOff();

ybar
xbar
sgn = 1;
if(xbar < -0.05)
    %system.executeTrajectoryToRelativePose(-0.05,0,pi(),sgn);
    ybar = -ybar;
    xbar = -xbar;
    th = th - pi;
    robot.moveAng(0.1, pi);
    pause(0.75);
    robot.move(0,0);
end
deg = th * 180 / pi
xbar = xbar - (robot.length + robot.palletL)*cos(th)
ybar = ybar - (robot.length + robot.palletL)*sin(th)
if(numel(yArray) == 0)
    beep;
else
    system.executeTrajectoryToRelativePose(xbar,ybar,th,sgn);
end