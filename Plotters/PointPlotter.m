classdef PointPlotter
    properties
        pointX = 0;
        pointY = 0;
        
        fig;
    end
    
    methods
        function obj = PointPlotter()
            obj.fig = figure();
        end
        
        function obj = UpdatePoints(obj, x, y)
            obj.pointX = x;
            obj.pointY = y;
            PlotArrays(obj);
        end
        
        function PlotArrays(obj)
            figure(obj.fig);
            clf;
            plot(0,0, 'ks', obj.pointX, obj.pointY, 'ro');
            axis([-100,100,-100,100])
        end
    end
    
end

