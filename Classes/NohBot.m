classdef NohBot<handle
    %   General controller class for robot
    
    properties
        %% Final parameters
        width = 8.9/100;
        offset = 10/100;
        thoffset = 5 * pi/180;
        delay = 0.2;
        
        %delay = 0.575;
        
        % Initialized in the constructor
        startTic
        
        %% Live parameters
        % Tic from the previous update
        prevTic = 0;
        % Toc from prevTic
        deltaTime = 0;
        
        % Current positions relative to starting position
        % Defined in world frame, forward is y+, right is x+
        x = 0;
        y = 0;
        th = 0;
        
        %% Objects
        % Actual ROS robot object
        rasp
    end
    
    %% Static Calculations
    methods(Static)
        function [x,y] = irToxy(i,r)
            x = cosd(i) * r;
            y = sind(i) * r;
        end
        
        function angVel = getAngVel(velocity,bearing,dist)
            if(bearing == 0)
                angVel =0;
                return;
            end
            
            theta = 2*bearing;
            r = sqrt(dist*dist/(2-2*cosd(theta)));
            curv = 1/r;
            angVel = curv * velocity;
        end
        
    end
    
    methods
        %% Initialization
        function obj = NohBot()
            close all;
            %rosshutdown();
            pause(0.1);
            
            obj.rasp = raspbot();
            pause(0.2);
            sendVelocity(obj.rasp,0,0)
            obj.startTic = tic;

            global encoderData;
            global encoderTime;
            global laserTime;
            global sTime;

            sTime = double(obj.rasp.encoders.LatestMessage.Header.Stamp.Sec) + double(obj.rasp.encoders.LatestMessage.Header.Stamp.Nsec)/1e9;
            enc = zeros(1,2);
            enc(1) = obj.rasp.encoders.LatestMessage.Vector.X;
            enc(2) = obj.rasp.encoders.LatestMessage.Vector.Y;
            encoderData = enc;
            encoderTime = sTime;

            laserTime = sTime;

            obj.rasp.encoders.NewMessageFcn =@EncoderListener;
            obj.rasp.laser.NewMessageFcn = @LaserListener;
        end
                
        %% Calculation
        function [velL, velR] = angVelToWheel(obj, vel, angVel)
            velL = vel - angVel*obj.width/2;
            velR = vel + angVel*obj.width/2;
        end
        
        function [vel, angVel] = wheelToAngVel(obj, vl, vr)
            angVel = (vr - vl)/obj.width;
            vel = (vr+vl)/2;
        end
        
        function [x, y, th] = modelDiffSteerRobot(obj, vl, vr, t0, tf, dt )
            th = obj.th;
            x = obj.x;
            y = obj.y;
            angVel = wheelToAngVel(obj, vl, vr);
            V = (vl + vr)/2;
            while(t0 < tf)
               th = th + angVel*dt/2;
               x = x + V*cos(th)*dt;
               y = y + V*sin(th)*dt;
               th = th + angVel*dt/2;
            end    
        end

        %% Command Wrappers
        function move(obj, leftVel, rightVel)
            sendVelocity(obj.rasp, rightVel, leftVel);
            pause(0.001);
        end
        
        function moveAng(obj, vel, angVel)
            [vl, vr] = obj.angVelToWheel(vel, angVel);
            obj.move(vl, vr);
            pause(0.001);
        end
        
        function laserOn(obj)
            obj.rasp.startLaser();
            pause(3);
        end
        
        function laserOff(obj)
            obj.rasp.stopLaser();
        end
        
        function stop(obj)
            obj.rasp.stop();
        end

    end
end

