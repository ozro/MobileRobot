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
            index = i-1;
        end
    end
    clc
    disp(min)
    x = min * cosd(index);
    y = min * sind(index);
    UpdatePoints(plot,x,y);
    
    if(min < 1)
        robot.sendVelocity(-0.15*(1-min),-0.15*(1-min));
    end
    
    omega = findOmega(0.15,index,min);
    if (index>90)
        omega = omega * (-1);
    end
    vr = 0.15 + omega*0.0445
    vl = 0.15 - omega*0.0445
    robot.sendVelocity(vl, vr);
    pause(0.2)
end

robot.stopLaser()