classdef NohBot<handle
    %   General controller class for robot
    
    properties
        %% Final parameters
        width
        indexSpacing
        forwardIndex
        
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
        
        % Instantaneous velocities in each wheel
        velL = 0;
        velR = 0;
        
        avgVel = 0;
        angVel = 0;
        
        %% Data
        % Logs are synced to timeLog
        timeLog = zero(1,1);
        encoderLog = zero(1,2); % L, R
        posLog = zero(1,2); % x, y
        velLog = zero(1,2); % L, R
        avgVelLog = zero(1,1);
        angVelLog = zero(1,1);
        
        %% Objects
        % Actual ROS robot object
        rasp
    end
    
    methods
        %% Initialization
        function obj = NohBot()
            rosshutdown();
            obj.rasp = raspbot();
            sendVelocity(obj.rasp,0,0)
            obj.startTic = tic;
        end
        
        function obj = restart(obj)
            obj.startTic = tic;
            obj.prevTic = tic;
            obj.deltaTime = 0;
            
            obj.xPos = 0;
            obj.yPos = 0;
            obj.yaw = pi/2;
            obj.encoderL = 0;
            obj.encoderR = 0;
            obj.velL = 0;
            obj.velR = 0;
            obj.avgVel = 0;
            obj.angVel = 0;
            
            obj.timeLog = zero(1,1);
            obj.encoderLog = zero(1,2);
            obj.posLog = zero(1,2);
            obj.velLog = zero(1,2);
            obj.avgVelLog = zero(1,1);
            obj.angVelLog = zero(1,1);
        end
        
        %% Calculation
        function [x, y, b] = irToXy(obj, i, r)
            b = (i - obj.forwardIndex)*obj.indexSpacing;
            absBearing = obj.yaw + b;
            x = cos(absBearing) * r;
            y = sin(absBearing) * r;
        end
        
        %% Wrappers
        function moveRobot(obj, leftVel, rightVel)
            sendVelocity(obj.robot, leftVel, rightVel);
        end
        
        function startLaser(obj)
            obj.rasp.startLaser();
            pause(3);
        end
        
        function stopLaser(obj)
            obj.rasp.stopLaser();
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
            obj.encoderL = obj.rasp.encoders.LatestMessage.Data(1);
            obj.encoderR = obj.rasp.encoders.LatestMessage.Data(2);
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

