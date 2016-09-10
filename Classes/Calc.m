classdef Calc
    methods(Static)
        function [x,y] = irToxy(i,r)
            x = cosd(i) * r;
            y = sind(i) * r;
        end
        function angVel = getAngVel(velocity,bearing,dist)
            if(bearing == 0)
                angVel =0;
                return;
            end
            
            theta = 2*bearing;
            r = sqrt(dist*dist/(2-2*cosd(theta)));
            curv = 1/r;
            angVel = curv * velocity;
        end
    end
    
end

