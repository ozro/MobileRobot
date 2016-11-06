classdef stateEstimator < handle
    %STATEESTIMATOR Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        nohbot
        odoPose
        fusePose
        prevEnc
        prevT
        LML
    end
    
    methods
        function obj = stateEstimator(pose, nohbot)
            p1 = [0,      0; 
                  0, 1.2192];
            p2 = [1.2192, 0; 
                  0, 0];
            obj.LML = LineMapLocalizer(p1, p2, 0.01, 0.001, 0.0005);
            global encoderData
            global encoderTime
            obj.prevEnc = encoderData;
            obj.prevT = encoderTime;
            obj.odoPose = pose;
            obj.fusePose = pose;
            obj.nohbot = nohbot;
        end
        
        function processOdometryData(obj)
            global encoderData;
            global encoderTime;
            
            dt = encoderTime - obj.prevT;
            dEnc = encoderData - obj.prevEnc;
            
            v = dEnc / dt;
            V = (v(1)+v(2))/2;
            p = obj.odoPose.getPoseVec();
            x = p(1);
            y = p(2);
            th = p(3);
            angVel = obj.nohbot.wheelToAngVel(v(1), v(2));
            th = th + angVel*dt/2;
            
            x = x + V*cos(th)*dt;
            y = y + V*sin(th)*dt;
            th = th + angVel*dt/2;
            
            obj.odoPose = pose(x,y,th);

            p = obj.fusePose.getPoseVec()';

            x = p(1);
            y = p(2);
            th = p(3);
            angVel = obj.nohbot.wheelToAngVel(v(1), v(2));
            th = th + angVel*dt/2;
            
            x = x + V*cos(th)*dt;
            y = y + V*sin(th)*dt;
            th = th + angVel*dt/2;
            
            obj.fusePose = pose(x,y,th);
            
        end
        
        function processRangeImage(obj)
            global laserRanges
            if(size(laserRanges,1) == 0) 
                return; 
            end
            rangeImg = laserRanges;
            range = 1:10:360;
            rangeImg = rangeImg(range');
            goodOnes = rangeImg > 0.06 & rangeImg < 4.0;
            rangeImg = rangeImg(goodOnes);
            indices = (1:10:360)';
            indices = indices(goodOnes);    
            x = cosd(indices).*rangeImg;
            y = sind(indices).*rangeImg;
            modelPts = [x';y';ones(1,size(indices, 1))];

            [success, finalPose] = obj.LML.refinePose(obj.fusePose, modelPts, 15);
            if success
                [x,y,th] = obj.fusePose.getPoseVec();
                [fx,fy,fth] = finalPose.getPoseVec();
                dth = atan2(sin(fth-th),cos(fth-th));
                obj.fusePose = [fx-x, fy-y, dth]*0.25+[x, y, th];
            end
        end
       
    end
    
end

