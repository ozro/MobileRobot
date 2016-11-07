classdef LineMapLocalizer < handle
    %mapLocalizer A class to match a range scan against a map in
    % order to find the true location of the range scan relative to
    % the map.

    properties(Constant)
        maxErr = 0.05; % 5 cm
        minPts = 5; % min # of points that must match
    end

    properties(Access = private)
    end

    properties(Access = public)
        lines_p1 = [];
        lines_p2 = [];
        gain = 0.01;
        errThresh = 0.001;
        gradThresh = 0.0005;
    end
    
    methods
        function obj = LineMapLocalizer(lines_p1,lines_p2,gain,errThresh,gradThresh)
            % create a lineMapLocalizer
            obj.lines_p1 = lines_p1;
            obj.lines_p2 = lines_p2;
            obj.gain = gain;
            obj.errThresh = errThresh;
            obj.gradThresh = gradThresh;
        end
        
        function ro2 = closestSquaredDistanceToLines(obj,pi)
            % Find the squared shortest distance from pi to any line
            % segment in the supplied list of line segments.
            % pi is an array of 2d points
            % throw away homogenous flag
            pi = pi(1:2,:);
            r2Array = zeros(size(obj.lines_p1,2),size(pi,2));
            for i = 1:size(obj.lines_p1,2)
                [r2Array(i,:) , ~] = ClosestPOLS(pi,...
                obj.lines_p1(:,i),obj.lines_p2(:,i));
            end
            ro2 = min(r2Array,[],1);
        end
        
        function ids = throwOutliers(obj,pose,ptsInModelFrame)
            % Find ids of outliers in a scan.
            worldPts = pose.bToA()*ptsInModelFrame;
            r2 = obj.closestSquaredDistanceToLines(worldPts);
            ids = find(sqrt(r2) > obj.maxErr);
        end
        
        function avgErr = fitError(obj,pose,ptsInModelFrame)
            % Find the standard deviation of perpendicular distances of
            % all points to all lines
            % transform the points
            worldPts = pose.bToA()*ptsInModelFrame;

            r2 = obj.closestSquaredDistanceToLines(worldPts);
            r2(r2 == Inf) = [];
            err = sum(r2);
            num = length(r2);
            if(num >= LineMapLocalizer.minPts)
                avgErr = sqrt(err)/num;
            else
            % not enough points to make a guess
                avgErr = inf;
            end
        end
        
        function [errPlus0,J] = getJacobian(obj,poseIn,modelPts)
            % Computes the gradient of the error function
            errPlus0 = fitError(obj,poseIn,modelPts);

            eps = 0.001;
            dp = [eps; 0.0; 0.0];
            newPose = pose(poseIn.getPoseVec()+dp);

            % Fill me in?
            parX = fitError(obj,newPose,modelPts);
            dp = [0.0; eps; 0.0];
            newPose = pose(poseIn.getPoseVec()+dp);
            parY = fitError(obj,newPose,modelPts);
            dp = [0.0; 0.0; eps];
            newPose = pose(poseIn.getPoseVec()+dp);
            parZ = fitError(obj,newPose,modelPts);
            J= ([parX,parY,parZ]-errPlus0) ./ eps;
        end
        
        function [success, outPose] = refinePose(obj, inPose, modelPts, maxIters)
            %outPose = inPose;
            dt = 0.015;
            epsilon = 0.008;
            ids = obj.throwOutliers(inPose, modelPts);
            modelPts(:,ids) = [];
            success = false;
            for i=1:maxIters
                [e, J] = obj.getJacobian(inPose,modelPts);
                temp = isnan(inPose.getPoseVec());
                if temp(1)
                    success = false;
                    break;
                end
                if(e<epsilon)
                    success = true;
                    break;
                end
                inPose = pose((inPose.getPoseVec()' - J * dt)');
%                 worldPts = inPose.bToA() * modelPts;
%                 p = inPose.getPoseVec();
                
%                 g1 = [-5,5,12*0.0254, 12*0.0254 , -5, 5, 24*0.0254, 24*0.0254];
%                 g2 = [12*0.0254, 12*0.0254, -5, 5, 24*0.0254, 24*0.0254, -5, 5];
%                 plot(obj.lines_p1, obj.lines_p2, '-.ok', g1, g2, ':ob', worldPts(1,:), worldPts(2,:), 'm*', p(1), p(2), 'sr');      
%                 xlim([-0.5, 1.5]);
%                 ylim([-0.5, 1.5]);
%                 title('Scan Matching');
            end   
%             temp = isnan(inPose.getPoseVec());
%             if ~temp(1)
%                 outPose = inPose;
%             end
            outPose = inPose;
        end
    end
    
end

