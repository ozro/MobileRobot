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
            p1 = [       0,        0, 8*0.3048 ; 
                         0, 8*0.3048, 8*0.3048];
            p2 = [8*0.3048,        0, 8*0.3048 ; 
                         0,        0,        0];
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
            
            t = encoderTime;
            enc = encoderData;
            dt = t - obj.prevT;
            dEnc = enc - obj.prevEnc;
            
            if(dt == 0)
                return;
            end
            
            obj.prevT = t;
            obj.prevEnc = enc;
            
            v = dEnc / dt;
            
            p = obj.odoPose.getPoseVec();
            x = p(1);
            y = p(2);
            th = p(3);
            [V, angVel] = obj.nohbot.wheelToAngVel(v(1), v(2));
            th = th + angVel*dt/2;
            x = x + V*cos(th)*dt;
            y = y + V*sin(th)*dt;
            th = th + angVel*dt/2;
            obj.odoPose = pose(x,y,th);

            p = obj.fusePose.getPoseVec();
            x = p(1);
            y = p(2);
            th = p(3);
            th = th + angVel*dt/2;
            x = x + V*cos(th)*dt;
            y = y + V*sin(th)*dt;
            th = th + angVel*dt/2;
            if(th>pi)
                th = pi-th;
            end
            obj.fusePose = pose(x,y,th);
        end
        
        function processRangeImage(obj)
            global laserRanges
            if(size(laserRanges,1) == 0) 
                return; 
            end
            rangeImg = laserRanges;
            range = 1:6:360;
            rangeImg = rangeImg(range');
            goodOnes = rangeImg > 0.06 & rangeImg < 12*0.3048;
            rangeImg = rangeImg(goodOnes);
            indices = (1:6:360)';
            indices = indices(goodOnes);
            x = cosd(indices).*rangeImg;
            y = sind(indices).*rangeImg;
            modelPts = [x';y';ones(1,size(indices, 1))];
            
            [success, finalPose] = obj.LML.refinePose(obj.fusePose, modelPts, 20);
            if success
                p = obj.fusePose.getPoseVec();
                x = p(1);
                y = p(2);
                th = p(3);
                p = finalPose.getPoseVec();
                fx = p(1);
                fy = p(2);
                fth = p(3);
                dth = angdiff(th, fth);
                k = 0.25;
                newTh = k*dth + th;
                if(newTh>pi)
                    newTh = pi-newTh;
                end
                if(abs(fx-x)*k < 2)
                    obj.fusePose = pose((fx-x)*k + x, (fy-y)*k +y, newTh);
                end
            end
        end
       
    end
    
end

