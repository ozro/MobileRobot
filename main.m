% Lab02
%% Connect to the Robot
close all;

rosshutdown();
robot = raspbot();%Make sure you are connected to Raspbot-X WiFi network.

%% Test movements
%robot.sendVelocity(.050, .050)
%pause(.5)
%robot.sendVelocity(-.050, .050)
%pause(.5)
%robot.encoders.LatestMessage.Data(1)


%% Laser

% spin up the laser, wait for it to start spewing data
robot.startLaser()
pause(3)

plot = PointPlotter();

% look at the ranges (there are 360 of them)
while(true)
    %theta = 0:359
    %cosine = cosd(theta)
    %sine = sind(theta)
    r = robot.laser.LatestMessage.Ranges;
    %plot(r'.*cosine,r'.*sine,'+')
    min = 1.6;
    index = 1;
    for i = [1:91,270:360]
        if (r(i)<=1.5 && r(i)>=0.6 && r(i)<min )
            min = r(i);
            index = i;
        end
    end
    clc
    disp(min)
    x = min * cosd(index);
    y = min * sind(index);
    UpdatePoints(plot,x,y);
    
    robot.sendVelocity((min-1)/2, (min-1)/2);
    pause(0.2)
end

robot.stopLaser()