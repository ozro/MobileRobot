%clear all;
%robot = NohBot();

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
sgn = 1;
if(xbar < -0.05)
    th = th - pi;
end
thoffset = th - 40*180/pi;
deg = th * 180 / pi
xbar = xbar - (robot.offset)*cos(thoffset)
ybar = ybar - (robot.offset)*sin(thoffset)
if(numel(yArray) == 0)
    beep;
else
    system.executeTrajectoryToRelativePose(xbar,ybar,th,sgn);
end