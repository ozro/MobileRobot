classdef controller
    
    properties
        robot
        ref
        traj
        feedback
    end
    
    methods
        function obj = controller(Nohbot, ref, traj, feedback)
            obj.ref = ref;
            obj.traj = traj;
            obj.robot = Nohbot;
            obj.feedback = feedback;
        end
        
        function run(obj)
            t = 0;
            prevt = 0;
            start = tic;
            
            tf = obj.ref.getTrajDuration() + 1.5;

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
            while(t<tf+1)
                if(t == 0)
                    t = toc(start);
                    continue;
                end
                t = toc(start);
                %dt = t- prevt;
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

                realPoseW = [x, y, b];
                refPoseW = obj.traj.getPose(t-obj.robot.delay);

                ePoseW = refPoseW - realPoseW;
                H = [cos(b), -sin(b); sin(b), cos(b)];
                ePosW = [ePoseW(1); ePoseW(2)];
                ePosR = H\ePosW;
                ePoseR = [ePosR(1), ePosR(2), ePoseW(3)];
                [eV, eW] = controller.computeError(ePoseR);
                if(isnan(eV) || isnan(eW))
                    eV = 0;
                    eW = 0;
                end
                
                [refvel, refangvel] = obj.ref.computeControl(t);
                
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
                figure(1);
                title('Trajectory vs time')
                a1 = plot(timeArray, refArray(:,1), timeArray, realArray(:,1),timeArray, refArray(:,2), timeArray, realArray(:,2),timeArray, refArray(:,3), timeArray, realArray(:,3));
                legend(a1, 'ref x', 'real x', 'ref y', 'real y', 'ref th', 'real th');
                figure(2);
                title('Position graph')
                a2 = plot(refArray(:,1), refArray(:,2), realArray(:,1), realArray(:,2));
                legend(a2, 'ref', 'real');
                figure(3);
                title('Error vs Time');
                a3 = plot(timeArray, errorArray(:,1),timeArray, errorArray(:,2),timeArray, errorArray(:,3));
                legend(a3, 'x', 'y', 'th');
        end
    end
    methods(Static)
        function [eV, eW] = computeError(ePose)
            ePos = [ePose(1); ePose(2)];
            eb = ePose(3);
            kx = 0.65;
            ky = 0.45;
            kb = 0.0020;

            k = [kx, 0; 0, ky];
            u = k * ePos;
            eV = u(1);
            eW = u(2) + eb*kb;
        end
    end
end

