classdef mrplSystem
    %MRPLSYSTEM Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        robot
        feedback
    end
    
    methods 
        function obj = mrplSystem(Nobot,feedback)
            obj.robot = Nobot;
            obj.feedback = feedback;
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

            x = 0;
            y = 0;
            b = 0;
            prevV = 0;
            prevW = 0;
            
            timeArray = zeros(1,1);
            refArray = zeros(1, 3);
            realArray = zeros(1, 3);
            errorArray = zeros(1, 3);
            
            prevPos = zeros(2,1);
            prevB = 0;
            prevt = 0;
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
                refPoseW = getPoseAtTime(curve,t-obj.robot.delay);

                ePoseW = refPoseW - realPoseW;
                th = refPoseW(3);
                H = [cos(th), -sin(th); sin(th), cos(th)];
                ePosW = [ePoseW(1); ePoseW(2)];
                ePosR = H\ePosW;
                ePoseR = [ePosR(1), ePosR(2), ePoseW(3)];
                if(obj.feedback)
                    ePos = [ePoseR(1); ePoseR(2)];
                    eb = ePoseR(3);
                    kx = 0.25;
                    ky = 0.25;
                    kb = 0.0020;

                    k = [kx, 0; 0, ky];
                    u = k * ePos;
                    eV = u(1);
                    eW = u(2) + eb*kb;
                    
                    dPos = (ePos - prevPos)/dt;
                    prevPos = ePos;
                    deb = (eb - prevB)/dt;
                    prevB=eb;
                    
                    kdx = 0.015;
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
                
                timeArray(end + 1) = realT - sTime;
                realArray(end + 1, :) = realPoseW;
                refArray(end + 1, :) = refPoseW;
                errorArray(end + 1, :) = ePoseR;
               
            end
                obj.robot.stop();
                
                figure();
                plot(refArray(:,1), refArray(:,2), realArray(:,1), realArray(:,2));
                p = gcf;
                n = int2str((p.get('Number')+1)/2);
                title(strcat('Position Graph(', n, ')'));
                legend('ref','real');
                xlabel('x (m)');
                ylabel('y (m)');
                
                figure();
                plot(timeArray, errorArray(:,1),timeArray, errorArray(:,2),timeArray, errorArray(:,3));
                title(strcat('Error vs Time(', n, ')'));
                legend('x', 'y', 'th'); 
                xlabel('time (s)');
                ylabel('error (m)');
        end
        
        function executeTrajectoryToRelativePose(obj,x,y,th,sgn)
            curve = cubicSpiral.planTrajectory(x,y,th,sgn);
            curve.planVelocities(0.25);
            obj.executeTrajectory(curve);         
        end
    end
    
end

