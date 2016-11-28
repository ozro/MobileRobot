classdef mrplSystem<handle
    %MRPLSYSTEM Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        robot
        feedback
        plotflag
        refPoseW
        timeArray
        refArray
        realArray
        errorArray
        index
        est
    end
    
    methods 
        function obj = mrplSystem(Nobot,est, feedback, plotflag, pose)
            obj.robot = Nobot;
            obj.est = est;
            obj.feedback = feedback;
            obj.plotflag = plotflag;
            obj.refPoseW = pose;
                        
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
                obj.refPoseW = tran * [refPoseR(1); refPoseR(2); 1];
                obj.refPoseW(3) = refPoseR(3) + obj.startPose(3);

                ePoseW = obj.refPoseW - realPoseW;

                th = obj.refPoseW(3);
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
                obj.refArray(obj.index, :) = obj.refPoseW;
                obj.errorArray(obj.index, :) = ePoseR;
                obj.index = obj.index +1;
                pause(0.1);
            end
            obj.robot.stop();


            if(obj.plotflag)
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
        end
        
        function executeTrajectorySE(obj,curve, SE)
            start = tic;
            t = toc(start);
            
            tf = getTrajectoryDuration(curve);
            global sTime
            global encoderTime           

            prevt = 0;
            
            tran = [cos(obj.refPoseW(3)), -sin(obj.refPoseW(3)), obj.refPoseW(1); 
                    sin(obj.refPoseW(3)), cos(obj.refPoseW(3)), obj.refPoseW(2); 0,0,1];
            startTh = obj.refPoseW(3);
                
            prevEPos = [0; 0];
            prevB = 0;
            
            while(t<tf + 0.5)
                realdT = encoderTime-sTime;
                t = toc(start);
                if(t == 0)
                    continue;
                end
                dt = t- prevt;
                prevt = t;

                obj.est.processOdometryData();
                if(SE) 
                    obj.est.processRangeImage();
                end
                realPoseW = obj.est.fusePose.getPoseVec();
                if(t - obj.robot.delay <= tf)
                    refPoseR = getPoseAtTime(curve,t - obj.robot.delay);
                    obj.refPoseW = tran * [refPoseR(1); refPoseR(2); 1];
                    obj.refPoseW(3) = refPoseR(3) + startTh;
                end

                ePoseW = obj.refPoseW - realPoseW;

                th = realPoseW(3);
                H = [cos(th), sin(th); -sin(th), cos(th)];
                ePosW = [ePoseW(1); ePoseW(2)];
                ePosR = H*ePosW;
                ePoseR = [ePosR(1), ePosR(2 ), ePoseW(3)];
                if(obj.feedback)
                    eb = ePoseR(3);
                    kx = 0.05;
                    ky = 5;
                    kb = 0.5;

                    k = [kx, 0; 0, ky];
                    u = k * ePosR;
                    eV = u(1);
                    eW = u(2) + eb*kb;
                    
                    dPos = (ePosR-prevEPos)/dt;
                    prevEPos = ePosR;
                    deb = (eb-prevB)/dt;
                    prevB = eb;
                    
                    kdx = 0.001;
                    kdy = 0.0005;
                    kdb = 0.0005;
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
                obj.timeArray(obj.index) = realdT;
                obj.realArray(obj.index, :) = realPoseW;
                obj.refArray(obj.index, :) = obj.refPoseW;
                obj.errorArray(obj.index, :) = ePoseR;
                obj.index = obj.index +1;
            end
            obj.robot.stop();

            if(obj.plotflag)
                figure(1);
                plot(obj.timeArray(1:obj.index-1), obj.errorArray(1:obj.index-1,1),obj.timeArray(1:obj.index-1), obj.errorArray(1:obj.index-1,2),obj.timeArray(1:obj.index-1),obj.errorArray(1:obj.index-1,3));
                title('Error vs Time');
                legend('x', 'y', 'th'); 
                xlabel('time (s)');
                ylabel('error (m)');
                
                figure(2);
                plot(obj.refArray(1:obj.index-1,1), obj.refArray(1:obj.index-1,2), obj.realArray(1:obj.index-1,1), obj.realArray(1:obj.index-1,2));
                title('Position Graph');
                legend('Reference Trajectory','Sensed Trajectory');
                xlabel('x (m)');
                ylabel('y (m)');
            end
        end
        function executeTrajectoryToAbsPose(obj,tarX,tarY,tarTh,sgn,vel, SE)
            x = obj.refPoseW(1);
            y = obj.refPoseW(2);
            th = obj.refPoseW(3);

%             refPose = obj.est.fusePose.getPoseVec();
%             x = refPose(1);
%             y = refPose(2);
%             th = refPose(3);
            
            bob = [x,y,th]
            f = obj.est.fusePose.getPoseVec()
            o = obj.est.odoPose.getPoseVec()
            
            xp = -(x*cos(th) + y*sin(th));
            yp = (x*sin(th) - y*cos(th));
            tran = [cos(th), sin(th), xp; -sin(th), cos(th), yp; 0, 0, 1];
            
            relpos = tran*[tarX; tarY; 1];
            relTh = tarTh - th;
            
            relX = relpos(1);
            relY = relpos(2);
                        
            executeTrajectoryToRelativePose(obj, relX, relY, relTh, sgn, vel, SE);
        end
        
        function executeTrajectoryToRelativePose(obj,x,y,th,sgn, vel, SE)
            curve = cubicSpiral.planTrajectory(x,y,th,sgn);
            curve.planVelocities(vel);
            obj.executeTrajectorySE(curve, SE);
        end
        
        function moveRel(obj, d)
            sgn = abs(d)/d;
            obj.robot.move(sgn*0.05, sgn*0.05);
            pause(d / 0.05);
            obj.robot.move(0, 0);
            obj.est.processOdometryData();
            obj.est.processRangeImage();
        end
        
        function turnTh(obj, dth)
            obj.robot.move(-dth * obj.robot.width / 2, dth * obj.robot.width / 2);
            pause(1-0.0025);
            obj.robot.move(0, 0);
            th = obj.refPoseW(3) + dth;
            if(th >= pi)
                th = pi - th;
            end
            obj.est.processOdometryData();
            obj.est.processRangeImage();
            obj.refPoseW(3) = th;
        end
        
        function blind(obj, robPose, objPose, v)
            %robPose = obj.est.fusePose.getPoseVec()
            relX = objPose(1) - robPose(1);
            relY = objPose(2) - robPose(2) - 35/100;
            relTh = atan2(relY, relX) - robPose(3);
            %th = atan2(relY, relX) - pi/2;
            d = sqrt(relX^2 + relY^2);
            
            obj.turnTh(relTh);
            pause(0.1)
            obj.moveRel(v,d)
            %obj.turnTh(pi/2 - atan2(relY, relX));
            %pause(0.1)
        end
    end
end
