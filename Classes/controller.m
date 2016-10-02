classdef controller
    
    properties
        robot
        ref
        traj
    end
    
    methods
        function obj = controller(Nohbot, ref, traj)
            obj.ref = ref;
            obj.traj = traj;
            obj.robot = Nohbot;
        end
        
        function run(obj)
            t = 0;
            prevt = 0;
            start = tic;
            
            tf = obj.ref.getTrajDuration() + 1;

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
            
            index = 0;
            timeArray = zeros(tf * 500,1);
            refArray = zeros(tf* 500, 3);
            realArray = zeros(tf* 500, 3);
            while(t<tf)
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
                ePosW = [ePoseW(1); ePoseW(2)]
                ePosR = H\ePosW
                ePoseR = [ePosR(1), ePosR(2), ePoseW(3)];
                [eV, eW, eB] = controller.computeError(ePoseR)
                
                [refvel, refangvel] = obj.ref.computeControl(t)
                
                vel = refvel+eV
                angvel = refangvel + eW
                obj.robot.moveAng(vel, angvel);
                
                index = index + 1;
                timeArray(index) = realT - sTime;
                realArray(index, :) = realPoseW;
                refArray(index, :) = refPoseW;
            end
            
            plot(timeArray, refArray(:,1), timeArray, realArray(:,1),timeArray, refArray(:,2), timeArray, realArray(:,2),timeArray, refArray(:,3), timeArray, realArray(:,3));
        end
    end
    methods(Static)
        function [eV, eW, eB] = computeError(ePose)
            ePos = [ePose(1); ePose(2)];
            eB = ePose(3);
            kx = 0.01;
            ky = 0.01;

            k = [kx, 0; 0, ky];
            u = k * ePos;

            eV = u(1);
            eW = u(2);
        end
    end
    
end

