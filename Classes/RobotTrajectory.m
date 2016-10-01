classdef RobotTrajectory
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        time
        distance
        vel
        angVel;
        pose
        dt = 0.005;
    end
    
    methods
        function obj = RobotTrajectory(t1, t2, ref)            
            t = 0;
            index = 1;
            n = int32((t2 - t1)/obj.dt);
            obj.time = zeros(n,1);
            obj.distance = zeros(n,1);
            obj.vel = zeros(n,1);
            obj.angVel = zeros(n,1);
            obj.pose = zeros(n, 3);
            
            while (t<t2)
                if(index == 1)
                    stime = tic;
                    index = index +1;
                    continue;
                end
                obj.time(index) = toc(stime);
                [v, w] = ref.computeControl(t);
                obj.vel(index) = v;
                obj.angVel(index) = w;
                prevV = obj.vel(index-1);
                prevW = obj.angVel(index-1);
                
                v = (v+prevV)/2;
                w = (w+prevW)/2;
                
                obj.distance(index) = obj.distance(index-1) + v*obj.dt;
                
                prevPose = obj.pose(index-1, :);
                prevb = prevPose(3);
                prevPos = [prevPose(1), prevPose(2)];
                
                b = prevb + w * obj.dt;
                Pos = [prevPos(1) + v*cos(b)*obj.dt, prevPos(2) + v*sin(b)*obj.dt];
                
                obj.pose(index, :) = [Pos(1), Pos(2), b];
                
                index = index +1;
                t = t+obj.dt;
            end
        end
        
        function pose = getPose(obj, t)
            index = t/obj.dt;
            pose = interp1(obj.pose, index);
        end
    end
end

