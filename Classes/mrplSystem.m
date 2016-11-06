classdef mrplSystem<handle
    %MRPLSYSTEM Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        robot
        feedback
        plot
        startPose
        finalPose
        timeArray
        refArray
        realArray
        errorArray
        index
    end
    
    methods 
        function obj = mrplSystem(Nobot,feedback, plot, pose)
            obj.robot = Nobot;
            obj.feedback = feedback;
            obj.plot = plot;
            obj.startPose = pose;
            obj.finalPose = pose;
                        
            obj.timeArray = zeros(10000,1);
            obj.refArray = zeros(10000, 3);
            obj.realArray = zeros(10000, 3);
            obj.errorArray = zeros(10000, 3);
            obj.index = 1;
            
        end
        
        function executeTrajectory(obj,curve)
            t = 0;
            start = tic;
            
            tf = getTrajectoryDuration(curve);

            global encoderData;
            global encoderTime;
            global sTime;

            prevEnc = encoderData;
            prevrealT = encoderTime;
            
            if(obj.index == 1) 
                x = 0;
                y = 0;
                b = 0;
            else
                x = obj.realArray(obj.index-1, 1);
                y = obj.realArray(obj.index-1, 2);
                b = obj.realArray(obj.index-1, 3);
            end
            prevV = 0;
            prevW = 0;

            
            prevPos = [x; y];
            prevB = b;
            prevt = 0;
            
            tran = [cos(obj.startPose(3)), -sin(obj.startPose(3)), obj.startPose(1); 
                    sin(obj.startPose(3)), cos(obj.startPose(3)), obj.startPose(2); 0,0,1];

            while(t<tf + 0.25)
                if(t == 0)
                    t = toc(start);
                    continue;
                end
                t = toc(start);
                dt = t- prevt;
                prevt = t;

                realT = encoderTime;
                enc = encoderData;
                if(realT == prevrealT)
                    pause(0.0001)
                    continue;
                end
                realdt = realT - prevrealT;
                prevrealT = realT;

                dl = enc(1) - prevEnc(1);
                dr = enc(2) - prevEnc(2);
                prevEnc = enc;
                
                vl = (dl)/ realdt;
                vr = (dr)/ realdt;

                [v,w] = obj.robot.wheelToAngVel(vl,vr);
                
                avgW = (w + prevW)/2;
                avgV = (v + prevV)/2;
                prevV = v;
                prevW = w;
                
                b = b + avgW * realdt/2;
                x = x + avgV * cos(b) * realdt;
                y = y + avgV * sin(b) * realdt;
                b = b + avgW * realdt/2;

                realPoseW = [x, y, b]';
                refPoseR = getPoseAtTime(curve,t-obj.robot.delay);
                refPoseW = tran * [refPoseR(1); refPoseR(2); 1];
                refPoseW(3) = refPoseR(3) + obj.startPose(3);

                ePoseW = refPoseW - realPoseW;

                th = refPoseW(3);
                H = [cos(th), sin(th); -sin(th), cos(th)];
                ePosW = [ePoseW(1); ePoseW(2)];
                ePosR = H*ePosW;
                ePoseR = [ePosR(1), ePosR(2), ePoseW(3)];
                if(obj.feedback)
                    eb = ePoseR(3);
                    kx = 0.005;
                    ky = 0.005;
                    kb = 0.0015;

                    k = [kx, 0; 0, ky];
                    u = k * ePosR;
                    eV = u(1);
                    eW = u(2) + eb*kb;
                    
                    dPos = (ePosR-prevPos)/dt;
                    prevPos = ePosR;
                    deb = (eb-prevB)/dt;
                    prevB = eb;
                    
                    kdx = 0.0;
                    kdy = 0.0;
                    kdb = 0.0;
                    kd = [kdx, 0; 0, kdy];
                    du = kd * dPos;
                    eV = eV + du(1);
                    eW = eW + du(2) + deb * kdb;
                    if(isnan(eV) || isnan(eW))
                        eV = 0;
                        eW = 0;
                    end
                end
                
                refvel = curve.getVAtTime(t-obj.robot.delay);
                refangvel = curve.getwAtTime(t-obj.robot.delay);
                
                if(obj.feedback)
                    vel = refvel + eV;
                    angvel = refangvel + eW;
                else
                    vel = refvel;
                    angvel = refangvel;
                end
                obj.robot.moveAng(vel, angvel);
                
                obj.timeArray(obj.index) = realT - sTime;
                obj.realArray(obj.index, :) = realPoseW;
                obj.refArray(obj.index, :) = refPoseW;
                obj.errorArray(obj.index, :) = ePoseR;
                obj.index = obj.index +1;
            end
            obj.robot.stop();


            if(obj.plot)
                figure(1);
                plot(-obj.refArray(1:obj.index-1,2), obj.refArray(1:obj.index-1,1), -obj.realArray(1:obj.index-1,2), obj.realArray(1:obj.index-1,1));
                p = gcf;
                n = int2str((p.get('Number')+1)/2);
                title(strcat('Position Graph(', n, ')'));
                legend('ref','real');
                xlabel('x (m)');
                ylabel('y (m)');

                figure(2);
                plot(obj.timeArray(1:obj.index-1), obj.errorArray(1:obj.index-1,1),obj.timeArray(1:obj.index-1), obj.errorArray(1:obj.index-1,2),obj.timeArray(1:obj.index-1),obj.errorArray(1:obj.index-1,3));
                title(strcat('Error vs Time(', n, ')'));
                legend('x', 'y', 'th'); 
                xlabel('time (s)');
                ylabel('error (m)');
            end
         %  plot(obj.timeArray(1:obj.index-1),obj.refArray(1:obj.index-1,1),obj.timeArray(1:obj.index-1),obj.realArray(1:obj.index-1,1));
            
            obj.startPose = obj.finalPose;
        end
        
        function executeTrajectorySE(obj,curve)
            t = 0;
            start = tic;
            
            tf = getTrajectoryDuration(curve);
            global sTime
            global encoderTime
            
            est = stateEstimator(pose(obj.startPose), obj.robot);
           
            
            if(obj.index == 1) 
                x = 0;
                y = 0;
                b = 0;
            else
                x = obj.realArray(obj.index-1, 1);
                y = obj.realArray(obj.index-1, 2);
                b = obj.realArray(obj.index-1, 3);
            end
            
            prevPos = [x; y];
            prevB = b;
            prevt = 0;
            
            tran = [cos(obj.startPose(3)), -sin(obj.startPose(3)), obj.startPose(1); 
                    sin(obj.startPose(3)), cos(obj.startPose(3)), obj.startPose(2); 0,0,1];

            while(t<tf + 0.25)
                realT = encoderTime-sTime;
                if(t == 0)
                    t = toc(start);
                    continue;
                end
                t = toc(start);
                dt = t- prevt;
                prevt = t;

                est.processOdometryData();
                est.processRangeImage();
                clc
                est.fusePose.getPoseVec()
                realPoseW = est.fusePose.getPoseVec(); 
                refPoseR = getPoseAtTime(curve,t-obj.robot.delay);
                refPoseW = tran * [refPoseR(1); refPoseR(2); 1];
                refPoseW(3) = refPoseR(3) + obj.startPose(3);

                ePoseW = refPoseW - realPoseW;

                th = refPoseW(3);
                H = [cos(th), sin(th); -sin(th), cos(th)];
                ePosW = [ePoseW(1); ePoseW(2)];
                ePosR = H*ePosW;
                ePoseR = [ePosR(1), ePosR(2), ePoseW(3)];
                if(obj.feedback)
                    eb = ePoseR(3);
                    kx = 0.005;
                    ky = 0.005;
                    kb = 0.0015;

                    k = [kx, 0; 0, ky];
                    u = k * ePosR;
                    eV = u(1);
                    eW = u(2) + eb*kb;
                    
                    dPos = (ePosR-prevPos)/dt;
                    prevPos = ePosR;
                    deb = (eb-prevB)/dt;
                    prevB = eb;
                    
                    kdx = 0.0;
                    kdy = 0.0;
                    kdb = 0.0;
                    kd = [kdx, 0; 0, kdy];
                    du = kd * dPos;
                    eV = eV + du(1);
                    eW = eW + du(2) + deb * kdb;
                    if(isnan(eV) || isnan(eW))
                        eV = 0;
                        eW = 0;
                    end
                end
                
                refvel = curve.getVAtTime(t-obj.robot.delay);
                refangvel = curve.getwAtTime(t-obj.robot.delay);
                
                if(obj.feedback)
                    vel = refvel + eV;
                    angvel = refangvel + eW;
                else
                    vel = refvel;
                    angvel = refangvel;
                end
                obj.robot.moveAng(vel, angvel);
                obj.timeArray(obj.index) = realT;
                obj.realArray(obj.index, :) = realPoseW;
                obj.refArray(obj.index, :) = refPoseW;
                obj.errorArray(obj.index, :) = ePoseR;
                obj.index = obj.index +1;
            end
            obj.robot.stop();


            if(obj.plot)
                figure(1);
                plot(-obj.refArray(1:obj.index-1,2), obj.refArray(1:obj.index-1,1), -obj.realArray(1:obj.index-1,2), obj.realArray(1:obj.index-1,1));
                p = gcf;
                n = int2str((p.get('Number')+1)/2);
                title(strcat('Position Graph(', n, ')'));
                legend('ref','real');
                xlabel('x (m)');
                ylabel('y (m)');

                figure(2);
                plot(obj.timeArray(1:obj.index-1), obj.errorArray(1:obj.index-1,1),obj.timeArray(1:obj.index-1), obj.errorArray(1:obj.index-1,2),obj.timeArray(1:obj.index-1),obj.errorArray(1:obj.index-1,3));
                title(strcat('Error vs Time(', n, ')'));
                legend('x', 'y', 'th'); 
                xlabel('time (s)');
                ylabel('error (m)');
            end
         %  plot(obj.timeArray(1:obj.index-1),obj.refArray(1:obj.index-1,1),obj.timeArray(1:obj.index-1),obj.realArray(1:obj.index-1,1));
            
            obj.startPose = obj.finalPose;
        end
        function executeTrajectoryToAbsPose(obj,tarX,tarY,tarTh,sgn)
            x = obj.finalPose(1);
            y = obj.finalPose(2);
            th = obj.finalPose(3);
            xp = -(x*cos(th) + y*sin(th));
            yp = (x*sin(th) - y*cos(th));
            tran = [cos(th), sin(th), xp; -sin(th), cos(th), yp; 0, 0, 1];
            
            relpose = tran*[tarX; tarY; 1];
            relAng = tarTh - th;
            
            relX = relpose(1);
            relY = relpose(2);
            relTh = relAng;
            
            obj.finalPose = [tarX, tarY, tarTh];

            
            executeTrajectoryToRelativePose(obj, relX, relY, relTh, sgn);
        end
        
        function executeTrajectoryToRelativePose(obj,x,y,th,sgn)
            curve = cubicSpiral.planTrajectory(x * sgn,y,th,sgn);
            curve.planVelocities(0.25);
            obj.executeTrajectorySE(curve);
        end
    end
end

