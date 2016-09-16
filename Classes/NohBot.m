classdef NohBot<handle
    %   General controller class for robot
    
    properties
        %% Final parameters
        width = 8.9/100;
        
        % Initialized in the constructor
        startTic
        
        %% Live parameters
        % Tic from the previous update
        prevTic = 0;
        % Toc from prevTic
        deltaTime = 0;
        
        % Current positions relative to starting position
        % Defined in world frame, forward is y+, right is x+
        xPos = 0;
        yPos = 0;
        
        % Current yaw measured in radians
        % 0 from East, CCW positive
        yaw = pi/2;
        
        % Current encoder values
        encoderL = 0;
        encoderR = 0;
        encoderT = 0;
        
        % Instantaneous velocities in each wheel
        velL = 0;
        velR = 0;
        
        avgVel = 0;
        angVel = 0;
        
        %% Data
        % Logs are synced to timeLog
        timeLog = zeros(1,1);
        encoderLog = zeros(1,2); % L, R
        posLog = zeros(1,2); % x, y
        velLog = zeros(1,2); % L, R
        avgVelLog = zeros(1,1);
        angVelLog = zeros(1,1);
        
        %% Objects
        % Actual ROS robot object
        rasp
    end
    
    methods
        %% Initialization
        function obj = NohBot()
            close all;
            rosshutdown();
            obj.rasp = raspbot();
            sendVelocity(obj.rasp,0,0)
            obj.startTic = tic;
        end
                
        %% Calculation
        function [velL, velR] = angVelToWheel(obj, angVel)
            velL = 0.15 - angVel*obj.width/2;
            velR = 0.15 + angVel*obj.width/2;
        end
        
        %% Command Wrappers
        function move(obj, leftVel, rightVel)
            sendVelocity(obj.rasp, leftVel, rightVel);
            pause(0.01);
        end
        
        function laserOn(obj)
            obj.rasp.startLaser();
            pause(3);
        end
        
        function laserOff(obj)
            obj.rasp.stopLaser();
        end
        
        function ranges = laserRanges(obj)
            ranges = obj.rasp.laser.LatestMessage.Ranges;
        end
        %% Update functions
        function obj = update(obj)
            obj.deltaTime = toc(obj.prevTic);
            obj.prevTic = tic;
            
            GetEncoders(obj)
            GetVel(obj)
            GetAvgVel(obj)
            GetAngVel(obj)
            GetPos(obj)
            GetYaw(obj) % Called after Pos to ensure Pos is calculated using correct yaw
            UpdateLogs(obj)
        end
        
        function obj = GetEncoders(obj)
            obj.encoderL = obj.rasp.encoders.LatestMessage.X;
            obj.encoderR = obj.rasp.encoders.LatestMessage.Y;
            obj.encoderT = double(robot.encoders.LatestMessage.Header.Stamp.Sec) + double(robot.encoders.LatestMessage.Header.Stamp.Nsec)/1e9;
        end
        
        function obj = GetVel(obj)
            [prevL, prevR] = obj.encoderLog(end,:);
            dispL = obj.encoderL - prevL;
            dispR = obj.encoderR - prevR;
            
            obj.velL = dispL/obj.deltaTime;
            obj.velR = dispR/obj.deltaTime;
        end
        
        function obj = GetAvgVel(obj)
            obj.avgVel = (obj.velL + obj.velR)/2;
        end
        
        function obj = GetAngVel(obj)
            obj.angVel = (obj.velR - obj.velL)/obj.width;
        end
        
        function obj = GetPos(obj)
            obj.xPos = obj.avgVel * cos(obj.yaw);
            obj.yPos = obj.avgVel * sin(obj.yaw);
        end
        
        function obj = GetYaw(obj)
            obj.yaw = obj.yaw + obj.angVel * obj.deltaTime;
        end        
        
        function obj = UpdateLogs(obj)
            obj.timeLog = cat(1, obj.timeLog, obj.timeLog(end,:)+obj.deltaTime);
            obj.encoderLog = cat(1, obj.encoderLog, [obj.encoderL, obj.encoderR]);
            obj.posLog = cat(1, obj.posLog, [obj.xPos, obj.yPos]);
            obj.velLog = cat(1, obj.velLog, [obj.velL, obj.velR]);
            obj.avgVelLog = cat(1, obj.avgVelLog, obj.avgVel);
            obj.angVelLog = cat(1, obj.angVelLog, obj.angVel);
        end
 
    end
end

